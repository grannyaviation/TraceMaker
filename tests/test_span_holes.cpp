// SPDX-License-Identifier: GPL-3.0-or-later
// Holes by their layer span (HDI): a laser via F.Cu-In1 or In1-In2 drills only the layers it joins, so copper on the
// other layers passes under its hole and two holes that share no layer never meet. KiCad does the same
// (drc_test_provider_copper_clearance.cpp testHoles: a via's hole counts on the layers of the via;
// drc_test_provider_hole_to_hole.cpp: blind and buried vias only matter when they share a layer, micro vias are never
// the tested hole). The fixture is the sensor_ts stack: F.Cu-In1-In2 as two micro vias at one point.
#include <catch2/catch_test_macros.hpp>

#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "drc/copper.hpp"
#include "drc/drc.hpp"
#include "io/kicad/board_reader.hpp"
#include "io/kicad/project_reader.hpp"
#include "route/obstacles.hpp"
#include "route/router.hpp"

using namespace tmk;
namespace fs = std::filesystem;

namespace {

constexpr Coord MM = 1'000'000;

// 20 x 20 mm, six layers: F.Cu In1 In2 In3 In4 B.Cu (indices 0..5). Net A (1) in class PWR_0A5, net B (2) in Default.
// The stacked via sits at (10, 10): two micro vias 0.30 / 0.15, F.Cu-In1 and In1-In2, net A.
std::string board_text(const std::string& extra) {
  return "(kicad_pcb (version 20240108) (generator \"pcbnew\")\n"
         "  (layers (0 \"F.Cu\" signal) (1 \"In1.Cu\" signal) (2 \"In2.Cu\" signal) (3 \"In3.Cu\" signal) (4 \"In4.Cu\" signal)\n"
         "    (31 \"B.Cu\" signal) (37 \"F.SilkS\" user) (44 \"Edge.Cuts\" user))\n"
         "  (net 0 \"\") (net 1 \"A\") (net 2 \"B\")\n"
         "  (gr_rect (start 0 0) (end 20 20) (layer \"Edge.Cuts\") (stroke (width 0.1) (type solid)))\n"
         "  (via micro (at 10 10) (size 0.3) (drill 0.15) (layers \"F.Cu\" \"In1.Cu\") (net 1))\n"
         "  (via micro (at 10 10) (size 0.3) (drill 0.15) (layers \"In1.Cu\" \"In2.Cu\") (net 1))\n" +
         extra + ")\n";
}

// The project: JLCPCB's HDI numbers. `hole_clearance` and `clearance` are the knobs a test turns.
struct Files {
  fs::path dir, pcb;
  Files(const std::string& name, const std::string& board, const std::string& dru = "", double hole_clearance = 0.25, double clearance = 0.1, double hole_to_hole = 0.24) {
    dir = fs::temp_directory_path() / ("tmk_span_holes_" + name + "_" + std::to_string(::getpid()));
    fs::remove_all(dir);
    fs::create_directories(dir);
    pcb = dir / "b.kicad_pcb";
    std::ofstream(pcb) << board;
    if (!dru.empty()) std::ofstream(dir / "b.kicad_dru") << dru;
    std::ofstream(dir / "b.kicad_pro")
        << R"({"board":{"design_settings":{"rules":{"min_clearance":)" << clearance << R"(,"min_track_width":0.1,"min_hole_clearance":)"
        << hole_clearance << R"(,"min_hole_to_hole":)" << hole_to_hole << R"(,"min_copper_edge_clearance":0.2,"min_microvia_diameter":0.3,)"
        << R"("min_microvia_drill":0.15,"min_via_diameter":0.3,"min_through_hole_diameter":0.15,"min_via_annular_width":0.05,)"
        << R"("allow_microvias":true,"allow_blind_buried_vias":true}}},)"
        << R"("net_settings":{"classes":[{"name":"Default","clearance":)" << clearance
        << R"(,"track_width":0.1,"via_diameter":0.45,"via_drill":0.3,"microvia_diameter":0.3,"microvia_drill":0.15},)"
        << R"({"name":"PWR_0A5","clearance":)" << clearance << R"(,"track_width":0.25}],)"
        << R"("netclass_patterns":[{"pattern":"A","netclass":"PWR_0A5"}]}})";
  }
  ~Files() { fs::remove_all(dir); }
};

int count(const drc::DrcReport& r, const std::string& type) {
  int n = 0;
  for (const auto& v : r.violations) n += v.type == type;
  return n;
}

// A 0.1 mm track of net B, 0.33 mm beside the stack's centre: 0.13 mm of air to the via's copper (clearance 0.1 holds),
// 0.205 mm to its hole (hole clearance 0.25 does not).
const geom::Point kA{9 * MM, 10 * MM + 330'000}, kB{11 * MM, 10 * MM + 330'000};
const Coord kW = 100'000;

}  // namespace

TEST_CASE("span holes: an In1-In2 micro via parses with its span", "[drc][route][span_holes]") {
  const Files f("parse", board_text(""));
  const auto lb = io::read_board_file(f.pcb.string());
  REQUIRE(lb.board.vias.size() == 2);
  CHECK(lb.board.vias[0].type == model::ViaType::Micro);
  CHECK(lb.board.vias[0].layer_top == 0);
  CHECK(lb.board.vias[0].layer_bottom == 1);
  CHECK(lb.board.vias[1].type == model::ViaType::Micro);
  CHECK(lb.board.vias[1].layer_top == 1);
  CHECK(lb.board.vias[1].layer_bottom == 2);
  CHECK(lb.board.copper_count() == 6);
}

TEST_CASE("span holes: a laser via's hole is on its own layers only", "[drc][route][span_holes]") {
  const Files f("mask", board_text(""));
  const auto lb = io::read_board_file(f.pcb.string());
  const auto cm = drc::build_copper(lb.board);
  REQUIRE(cm.holes.size() == 2);
  CHECK(cm.holes[0].layers == (model::layer_bit(0) | model::layer_bit(1)));
  CHECK(cm.holes[1].layers == (model::layer_bit(1) | model::layer_bit(2)));
}

TEST_CASE("span holes: a through hole is on every layer", "[drc][route][span_holes]") {
  const Files f("mask_pth", board_text(
      "  (footprint \"H\" (layer \"F.Cu\") (at 3 3) (property \"Reference\" \"H1\" (at 0 0) (layer \"F.SilkS\"))\n"
      "    (pad \"1\" thru_hole circle (at 0 0) (size 1 1) (drill 0.5) (layers \"*.Cu\") (net 2 \"B\")))\n"
      "  (footprint \"N\" (layer \"F.Cu\") (at 6 3) (property \"Reference\" \"H2\" (at 0 0) (layer \"F.SilkS\"))\n"
      "    (pad \"\" np_thru_hole circle (at 0 0) (size 0.8 0.8) (drill 0.8) (layers \"*.Cu\")))\n"
      "  (via (at 15 15) (size 0.45) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 2))\n"));
  const auto lb = io::read_board_file(f.pcb.string());
  const auto cm = drc::build_copper(lb.board);
  const model::LayerMask all = (model::LayerMask{1} << 6) - 1;
  REQUIRE(cm.holes.size() == 5);  // the plated pad, the NPTH, the two laser vias, the through via
  for (const auto& h : cm.holes)
    if (h.pad >= 0 || lb.board.vias[static_cast<std::size_t>(h.via)].type == model::ViaType::Through) CHECK(h.layers == all);
    else CHECK(h.layers != all);
}

TEST_CASE("span holes: the router lets copper pass under a laser via on the layers it does not drill", "[route][span_holes]") {
  const Files f("route_copper", board_text(""));
  auto lb = io::read_board_file(f.pcb.string());
  const auto rules = io::read_design_rules(f.pcb.string());
  route::Obstacles obs(lb.board, rules);
  // F.Cu, In1 and In2 carry the hole: the track keeps the hole clearance.
  for (int l = 0; l <= 2; ++l) CHECK(obs.segment_state(kA, kB, l, kW, 2, false) == 2);
  // In3, In4 and B.Cu do not: only the track clearance applies, and there is no copper.
  for (int l = 3; l <= 5; ++l) CHECK(obs.segment_state(kA, kB, l, kW, 2, false) == 0);
  // The cached path (fixed_code) agrees, for another net than the via's.
  const geom::Point p{10 * MM, 10 * MM + 330'000};
  for (int l = 0; l <= 2; ++l) CHECK(obs.fixed_code(p, l, kW / 2, 0, 2) != route::Obstacles::kFree);
  for (int l = 3; l <= 5; ++l) CHECK(obs.fixed_code(p, l, kW / 2, 0, 2) == route::Obstacles::kFree);
}

TEST_CASE("span holes: DRC flags a track against a laser via's hole only on its layers", "[drc][span_holes]") {
  const std::string tracks =
      "  (segment (start 9 10.33) (end 11 10.33) (width 0.1) (layer \"In2.Cu\") (net 2))\n"
      "  (segment (start 9 10.33) (end 11 10.33) (width 0.1) (layer \"In3.Cu\") (net 2))\n"
      "  (segment (start 9 10.33) (end 11 10.33) (width 0.1) (layer \"In4.Cu\") (net 2))\n"
      "  (segment (start 9 10.33) (end 11 10.33) (width 0.1) (layer \"B.Cu\") (net 2))\n";
  const Files f("drc_copper", board_text(tracks));
  const auto lb = io::read_board_file(f.pcb.string());
  const auto r = drc::run_drc(lb.board, io::read_design_rules(f.pcb.string()));
  CHECK(count(r, "hole_clearance") == 1);  // the In2 track
  CHECK(count(r, "clearance") == 0);
  for (const auto& v : r.violations)
    if (v.type == "hole_clearance") CHECK(v.layer == 2);
}

TEST_CASE("span holes: a through via keeps hole to hole from a laser via on a shared layer", "[route][drc][span_holes]") {
  // Copper clearance 0.05 and hole clearance 0.1, so that the 0.24 mm hole to hole is what decides: a 0.45 / 0.30
  // through via 0.45 mm away has 0.225 mm between the holes, 0.50 mm away 0.275 mm.
  const Files f("h2h_through", board_text(""), "", 0.1, 0.05);
  auto lb = io::read_board_file(f.pcb.string());
  const auto rules = io::read_design_rules(f.pcb.string());
  route::Obstacles obs(lb.board, rules);
  CHECK(obs.via_state({10 * MM + 450'000, 10 * MM}, 450'000, 300'000, 2, 0, false) == 2);
  CHECK(obs.via_state({10 * MM + 500'000, 10 * MM}, 450'000, 300'000, 2, 0, false) == 0);
  CHECK(obs.fixed_via_code({10 * MM + 450'000, 10 * MM}, 450'000, 300'000, 0, 2) == route::Obstacles::kBlocked);
  CHECK(obs.fixed_via_code({10 * MM + 500'000, 10 * MM}, 450'000, 300'000, 0, 2) != route::Obstacles::kBlocked);
  // The same pair in a board: DRC reports the through via against the laser vias.
  const Files g("h2h_drc", board_text("  (via (at 10.45 10) (size 0.45) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 2))\n"), "", 0.1, 0.05);
  const auto lg = io::read_board_file(g.pcb.string());
  const auto r = drc::run_drc(lg.board, io::read_design_rules(g.pcb.string()));
  CHECK(count(r, "hole_to_hole") >= 1);
}

TEST_CASE("span holes: holes that share no layer do not meet", "[route][drc][span_holes]") {
  const Files f("h2h_span", board_text(""), "", 0.1, 0.05);
  auto lb = io::read_board_file(f.pcb.string());
  const auto rules = io::read_design_rules(f.pcb.string());
  route::Obstacles obs(lb.board, rules);
  const geom::Point p{10 * MM + 300'000, 10 * MM};
  // A laser via In3-In4 0.30 mm beside the stack (F.Cu-In2): no shared layer, so neither the holes nor the copper meet.
  CHECK(obs.via_state_span(p, 300'000, 150'000, 2, 0, false, nullptr, 3, 4, model::ViaType::Micro) == 0);
  // One on In1-In2 does share layers.
  CHECK(obs.via_state_span(p, 300'000, 150'000, 2, 0, false, nullptr, 1, 2, model::ViaType::Micro) == 2);
  // DRC: a drilled blind via In3-In4 beside the stack. KiCad tests a blind via's hole against another via only on a
  // shared layer. The same blind via on In1-In2 meets the stack.
  const Files g("h2h_blind_apart", board_text("  (via blind (at 10.3 10) (size 0.45) (drill 0.3) (layers \"In3.Cu\" \"In4.Cu\") (net 2))\n"), "", 0.1, 0.05);
  const Files h("h2h_blind_shared", board_text("  (via blind (at 10.3 10) (size 0.45) (drill 0.3) (layers \"In1.Cu\" \"In2.Cu\") (net 2))\n"), "", 0.1, 0.05);
  const auto lg = io::read_board_file(g.pcb.string());
  const auto rg = drc::run_drc(lg.board, io::read_design_rules(g.pcb.string()));
  CHECK(count(rg, "hole_to_hole") == 0);
  CHECK(count(rg, "hole_clearance") == 0);
  const auto lh = io::read_board_file(h.pcb.string());
  const auto rh = drc::run_drc(lh.board, io::read_design_rules(h.pcb.string()));
  CHECK(count(rh, "hole_to_hole") >= 1);
}

TEST_CASE("span holes: the two halves of a stacked via are no co-located holes", "[drc][span_holes]") {
  // KiCad never tests micro vias against each other (hole-to-hole provider, line 165): the halves of one stack at one
  // point raise nothing.
  const Files f("colocated", board_text(""));
  const auto lb = io::read_board_file(f.pcb.string());
  const auto r = drc::run_drc(lb.board, io::read_design_rules(f.pcb.string()));
  CHECK(count(r, "holes_co_located") == 0);
  CHECK(count(r, "hole_to_hole") == 0);
}

TEST_CASE("span holes: connectivity joins the two halves of a stack and an In2 track from them", "[drc][span_holes]") {
  // P1 (F.Cu pad) - F.Cu track - stack (F.Cu-In1, In1-In2) - In2 track - P2 (through hole pad), all net A.
  const std::string chain =
      "  (footprint \"P\" (layer \"F.Cu\") (at 4 10) (property \"Reference\" \"P1\" (at 0 0) (layer \"F.SilkS\"))\n"
      "    (pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\") (net 1 \"A\")))\n"
      "  (footprint \"P\" (layer \"F.Cu\") (at 16 10) (property \"Reference\" \"P2\" (at 0 0) (layer \"F.SilkS\"))\n"
      "    (pad \"1\" thru_hole circle (at 0 0) (size 1 1) (drill 0.5) (layers \"*.Cu\") (net 1 \"A\")))\n"
      "  (segment (start 4 10) (end 10 10) (width 0.25) (layer \"F.Cu\") (net 1))\n"
      "  (segment (start 10 10) (end 16 10) (width 0.25) (layer \"In2.Cu\") (net 1))\n";
  {
    const Files f("conn", board_text(chain));
    const auto lb = io::read_board_file(f.pcb.string());
    const auto r = drc::run_drc(lb.board, io::read_design_rules(f.pcb.string()));
    CHECK(r.unconnected.empty());
  }
  {
    // Control: without the second half the In2 track has nothing to join.
    std::string one = board_text(chain);
    const std::string half = "  (via micro (at 10 10) (size 0.3) (drill 0.15) (layers \"In1.Cu\" \"In2.Cu\") (net 1))\n";
    one.erase(one.find(half), half.size());
    const Files f("conn_half", one);
    const auto lb = io::read_board_file(f.pcb.string());
    const auto r = drc::run_drc(lb.board, io::read_design_rules(f.pcb.string()));
    CHECK_FALSE(r.unconnected.empty());
  }
}

TEST_CASE("span holes: a (layer inner) disallow track rule with a NetClass condition is honoured", "[route][drc][span_holes]") {
  const char* dru = "(version 1)\n(rule \"pwr outer only\" (layer inner) (condition \"A.NetClass == 'PWR_0A5'\") (constraint disallow track))\n";
  const std::string tracks =
      "  (segment (start 3 3) (end 5 3) (width 0.1) (layer \"In2.Cu\") (net 1))\n"  // class PWR_0A5 on an inner layer: not allowed
      "  (segment (start 3 5) (end 5 5) (width 0.1) (layer \"F.Cu\") (net 1))\n"    // the same class on an outer layer: allowed
      "  (segment (start 3 7) (end 5 7) (width 0.1) (layer \"In3.Cu\") (net 2))\n"; // another class on an inner layer: allowed
  const Files f("netclass_rule", board_text(tracks), dru);
  const auto lb = io::read_board_file(f.pcb.string());
  const auto r = drc::run_drc(lb.board, io::read_design_rules(f.pcb.string()));
  CHECK(count(r, "items_not_allowed") == 1);
  // The router: a wall of outer-layer keepouts (x 9..11) forces both nets across on an inner layer. Net A is in
  // PWR_0A5 and may not use one; net B may. Without the rule both cross. (Obstacles::segment_state does not see the
  // rule: the router keeps it per net in Router::layer_ok, as for a NetName condition.)
  auto wall_board = []() {
    auto fp = [](const char* ref, double y, int net, const char* name) {
      return std::string("  (footprint \"R\" (layer \"F.Cu\") (at 4 ") + std::to_string(y) + ")\n    (property \"Reference\" \"" + ref +
             "a\" (at 0 0) (layer \"F.SilkS\"))\n    (pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\") (net " + std::to_string(net) +
             " \"" + name + "\")))\n  (footprint \"R\" (layer \"F.Cu\") (at 16 " + std::to_string(y) + ")\n    (property \"Reference\" \"" + ref +
             "b\" (at 0 0) (layer \"F.SilkS\"))\n    (pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\") (net " + std::to_string(net) + " \"" +
             name + "\")))\n";
    };
    return "(kicad_pcb (version 20240108) (generator \"pcbnew\")\n"
           "  (layers (0 \"F.Cu\" signal) (1 \"In1.Cu\" signal) (2 \"In2.Cu\" signal) (3 \"In3.Cu\" signal) (4 \"In4.Cu\" signal)\n"
           "    (31 \"B.Cu\" signal) (37 \"F.SilkS\" user) (44 \"Edge.Cuts\" user))\n"
           "  (net 0 \"\") (net 1 \"A\") (net 2 \"B\")\n"
           "  (gr_rect (start 0 0) (end 20 20) (layer \"Edge.Cuts\") (stroke (width 0.1) (type solid)))\n" +
           fp("R1", 5, 1, "A") + fp("R2", 15, 2, "B") +
           "  (zone (net 0) (net_name \"\") (layers \"F.Cu\" \"B.Cu\") (name \"wall\") (hatch edge 0.5) (connect_pads (clearance 0))\n"
           "    (min_thickness 0.25) (keepout (tracks not_allowed) (vias not_allowed) (pads allowed) (copperpour allowed) (footprints allowed))\n"
           "    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5)) (polygon (pts (xy 9 0) (xy 11 0) (xy 11 20) (xy 9 20))))\n)\n";
  };
  auto route = [&](const char* name, const char* rule) {
    const Files h(name, wall_board(), rule);
    const auto lh = io::read_board_file(h.pcb.string());
    route::RouterOptions o;
    o.work_budget = 4'000'000;
    o.gpu_device = -1;
    o.time_limit_s = 600;
    return route::Router(lh.board, io::read_design_rules(h.pcb.string()), o).run();
  };
  auto inner = [](const route::RouteResult& res, model::NetId net) {
    for (const auto& t : res.tracks)
      if (t.net == net && t.layer >= 1 && t.layer <= 4) return true;
    return false;
  };
  const auto free_run = route("route_free", "");
  CHECK(free_run.routed == 2);
  CHECK(inner(free_run, 1));
  const auto ruled = route("route_ruled", dru);
  CHECK(ruled.routed == 1);  // net A cannot cross the wall
  CHECK_FALSE(inner(ruled, 1));
  CHECK(inner(ruled, 2));
}

// The sensor's own laser-via rules, copied from sensor_ts kicad/sensor/sensor.kicad_dru (jlc-hdi-microvia-hole-to-hole,
// jlc-hdi-microvia-edge). KiCad's hole-to-hole rule has no net, so it binds a same-net through via too.
const char* kJlcMicroviaRules =
    "(version 1)\n"
    "(rule \"jlc-hdi-microvia-hole-to-hole\" (condition \"A.Via_Type == 'Micro' || B.Via_Type == 'Micro'\") (constraint hole_to_hole (min 0.24mm)))\n"
    "(rule \"jlc-hdi-microvia-edge\" (condition \"A.Via_Type == 'Micro'\") (constraint edge_clearance (min 0.35mm)))\n";

TEST_CASE("span holes: the sensor's microvia hole-to-hole rule binds a through via of the same net", "[route][drc][span_holes]") {
  // Board minimum 0.2 mm. A same-net 0.45 / 0.30 through via 0.45 mm away has 0.225 mm between the holes: legal by the
  // board minimum, short of the rule's 0.24 mm. 0.50 mm away (0.275 mm) is legal.
  const std::string via45 = "  (via (at 10.45 10) (size 0.45) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 1))\n";
  {
    // Without the rule the board minimum decides.
    const Files f("via_type_off", board_text(""), "", 0.25, 0.1, 0.2);
    auto lb = io::read_board_file(f.pcb.string());
    const auto rules = io::read_design_rules(f.pcb.string());
    route::Obstacles obs(lb.board, rules);
    CHECK(obs.via_state({10 * MM + 450'000, 10 * MM}, 450'000, 300'000, 1, 0, false) == 0);
  }
  const Files f("via_type", board_text(""), kJlcMicroviaRules, 0.25, 0.1, 0.2);
  auto lb = io::read_board_file(f.pcb.string());
  const auto rules = io::read_design_rules(f.pcb.string());
  route::Obstacles obs(lb.board, rules);
  CHECK(obs.rules().warnings().empty());
  CHECK(obs.via_state({10 * MM + 450'000, 10 * MM}, 450'000, 300'000, 1, 0, false) == 2);
  CHECK(obs.via_state({10 * MM + 500'000, 10 * MM}, 450'000, 300'000, 1, 0, false) == 0);
  CHECK(obs.fixed_via_code({10 * MM + 450'000, 10 * MM}, 450'000, 300'000, 0, 1) == route::Obstacles::kBlocked);
  CHECK(obs.fixed_via_code({10 * MM + 500'000, 10 * MM}, 450'000, 300'000, 0, 1) != route::Obstacles::kBlocked);
  // Two through vias 0.225 mm between holes stay legal: the rule names laser vias.
  const Files g("via_type_pair", board_text("  (via (at 5 5) (size 0.45) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 1))\n" +
                                            std::string("  (via (at 5.525 5) (size 0.45) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 1))\n")),
                 kJlcMicroviaRules, 0.25, 0.1, 0.2);
  const auto lg = io::read_board_file(g.pcb.string());
  CHECK(count(drc::run_drc(lg.board, io::read_design_rules(g.pcb.string())), "hole_to_hole") == 0);
  // DRC: the through via is flagged against both halves (the stack, F.Cu-In1 and In1-In2), as KiCad does.
  const Files h("via_type_drc", board_text(via45), kJlcMicroviaRules, 0.25, 0.1, 0.2);
  const auto lh = io::read_board_file(h.pcb.string());
  const auto r = drc::run_drc(lh.board, io::read_design_rules(h.pcb.string()));
  CHECK(count(r, "hole_to_hole") == 2);
  for (const auto& w : r.warnings) CHECK(w.find("not applied") == std::string::npos);
  // jlc-hdi-microvia-edge: 0.35 mm from the edge for a laser via only (the board minimum is 0.2 mm).
  const Files e("via_type_edge", board_text("  (via micro (at 0.45 5) (size 0.3) (drill 0.15) (layers \"F.Cu\" \"In1.Cu\") (net 1))\n"
                                            "  (via (at 0.45 15) (size 0.45) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 1))\n"),
                kJlcMicroviaRules, 0.25, 0.1, 0.2);
  const auto le = io::read_board_file(e.pcb.string());
  const auto re = drc::run_drc(le.board, io::read_design_rules(e.pcb.string()));
  CHECK(count(re, "copper_edge_clearance") == 1);
}

TEST_CASE("span holes: a laser via and a pad hole are one hole-to-hole pair per via, as in KiCad", "[drc][span_holes]") {
  // A plated 0.3 mm pad hole 0.45 mm from the stack: 0.225 mm between holes. KiCad's pad pass finds each laser via once
  // (a laser via never runs the via pass), so two violations for the two halves.
  const Files f("pad_pair", board_text(
      "  (footprint \"H\" (layer \"F.Cu\") (at 10.45 10) (property \"Reference\" \"H1\" (at 0 0) (layer \"F.SilkS\"))\n"
      "    (pad \"1\" thru_hole circle (at 0 0) (size 0.45 0.45) (drill 0.3) (layers \"*.Cu\") (net 2 \"B\")))\n"), "", 0.1, 0.05);
  const auto lb = io::read_board_file(f.pcb.string());
  const auto r = drc::run_drc(lb.board, io::read_design_rules(f.pcb.string()));
  CHECK(count(r, "hole_to_hole") == 2);
}

TEST_CASE("span holes: a routed laser via is an obstacle on its own layers only", "[route][span_holes]") {
  const Files f("routed_micro", board_text(""));
  auto lb = io::read_board_file(f.pcb.string());
  const auto rules = io::read_design_rules(f.pcb.string());
  route::Obstacles obs(lb.board, rules);
  model::Via v;
  v.pos = {5 * MM, 5 * MM};
  v.size = 300'000;
  v.drill = 150'000;
  v.layer_top = 0;
  v.layer_bottom = 1;
  v.type = model::ViaType::Micro;
  v.net = 1;
  lb.board.vias.push_back(v);
  obs.add_via(static_cast<int>(lb.board.vias.size()) - 1, 7);
  const geom::Point a{4 * MM, 5 * MM + 330'000}, b{6 * MM, 5 * MM + 330'000};
  for (int l = 0; l <= 1; ++l) {
    std::vector<int> owners;
    CHECK(obs.segment_state(a, b, l, kW, 2, true, &owners) == 1);  // rippable, not fixed
    CHECK(std::find(owners.begin(), owners.end(), 7) != owners.end());
    CHECK(obs.segment_state(a, b, l, kW, 2, false) == 2);
  }
  for (int l = 2; l <= 5; ++l) CHECK(obs.segment_state(a, b, l, kW, 2, true) == 0);
  // The search's routed-copper check agrees (its via parts take the layers of the probe).
  const auto probe = geom::Shape::segment(a, b, kW / 2);
  CHECK(obs.routed_state(probe, 1, 2, drc::ItemKind::Track, false, nullptr) == 2);
  CHECK(obs.routed_state(probe, 3, 2, drc::ItemKind::Track, false, nullptr) == 0);
  // A new micro via on In3-In4 beside it meets nothing: no shared layer, whatever the distance of the holes.
  CHECK(obs.via_state_span({5 * MM + 300'000, 5 * MM}, 300'000, 150'000, 2, 0, true, nullptr, 3, 4, model::ViaType::Micro) == 0);
  CHECK(obs.routed_via_state(geom::Shape::point({5 * MM + 300'000, 5 * MM}, 150'000), model::layer_bit(3) | model::layer_bit(4), 2, false, 75'000) == 0);
  CHECK(obs.routed_via_state(geom::Shape::point({5 * MM + 300'000, 5 * MM}, 150'000), model::layer_bit(0), 2, false, 75'000) == 2);
}

TEST_CASE("span holes: a (layer inner) track_width rule with a NetClass condition", "[drc][span_holes]") {
  const char* dru = "(version 1)\n(rule \"pwr inner width\" (layer inner) (condition \"A.NetClass == 'PWR_0A5'\") (constraint track_width (min 0.25mm)))\n";
  const std::string tracks =
      "  (segment (start 3 3) (end 5 3) (width 0.1) (layer \"In2.Cu\") (net 1))\n"  // class PWR_0A5, inner, 0.1 < 0.25: flagged
      "  (segment (start 3 5) (end 5 5) (width 0.1) (layer \"F.Cu\") (net 1))\n"     // outer: the rule does not apply
      "  (segment (start 3 7) (end 5 7) (width 0.1) (layer \"In3.Cu\") (net 2))\n"   // other class: not applied
      "  (segment (start 3 9) (end 5 9) (width 0.25) (layer \"In4.Cu\") (net 1))\n"; // wide enough
  const Files f("width_rule", board_text(tracks), dru);
  const auto lb = io::read_board_file(f.pcb.string());
  const auto r = drc::run_drc(lb.board, io::read_design_rules(f.pcb.string()));
  CHECK(count(r, "track_width") == 1);
  for (const auto& v : r.violations)
    if (v.type == "track_width") CHECK(v.layer == 2);
  // The router takes its width from the net class and does not read a track_width minimum from a rule: net A's class
  // width is 0.25 mm here, so its tracks meet the rule; a rule above the class width would not be met by the router.
}
