// SPDX-License-Identifier: GPL-3.0-or-later
// Stacked laser vias as router moves (HDI, doc 05 §21.1): with --stacked-vias a layer change F.Cu-In2 or B.Cu-In3 is two micro
// vias at one point (F.Cu-In1 plus In1-In2, or B.Cu-In4 plus In4-In3), as KiCad models a stacked via; --stagger puts the second
// one beside the first with a stub on the middle layer. The router holds what KiCad checks for them (micro-via rules, the 0.35 mm
// edge rule, hole to hole against drilled holes) and JLC's laser spacing, which KiCad does not check. Also here: rule-based
// track widths per layer (sd50-inner, usb90-inner), the via type of a router probe in rule conditions, and the return-aware via
// cost of a side change (TM-5, --return-via-cost).
#include <catch2/catch_test_macros.hpp>

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "drc/drc.hpp"
#include "drc/refill.hpp"
#include "drc/rule_engine.hpp"
#include "geom/shape.hpp"
#include "io/kicad/board_editor.hpp"
#include "io/kicad/board_reader.hpp"
#include "io/kicad/project_reader.hpp"
#include "route/obstacles.hpp"
#include "route/router.hpp"

using namespace tmk;
namespace fs = std::filesystem;

namespace {

constexpr Coord MM = 1'000'000;

std::string mm(double v) {
  std::ostringstream o;
  o << v;
  return o.str();
}

std::string pad_fp(const std::string& ref, double x, double y, const std::string& layer, int net, const std::string& name, double size = 0.6) {
  return "  (footprint \"P\" (layer \"" + layer + "\") (at " + mm(x) + " " + mm(y) + ")\n    (property \"Reference\" \"" + ref +
         "\" (at 0 0) (layer \"F.SilkS\"))\n    (pad \"1\" smd rect (at 0 0) (size " + mm(size) + " " + mm(size) + ") (layers \"" + layer +
         "\") (net " + std::to_string(net) + " \"" + name + "\")))\n";
}

std::string area(const std::string& name, const std::string& layer, double x0, double y0, double x1, double y1, bool tracks, bool vias) {
  return "  (zone (net 0) (net_name \"\") (layer \"" + layer + "\") (name \"" + name + "\") (hatch edge 0.5) (connect_pads (clearance 0))\n"
         "    (min_thickness 0.25) (keepout (tracks " + (tracks ? "not_allowed" : "allowed") + ") (vias " + (vias ? "not_allowed" : "allowed") +
         ") (pads allowed) (copperpour allowed) (footprints allowed))\n    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5)) (polygon (pts (xy " +
         mm(x0) + " " + mm(y0) + ") (xy " + mm(x1) + " " + mm(y0) + ") (xy " + mm(x1) + " " + mm(y1) + ") (xy " + mm(x0) + " " + mm(y1) + "))))\n";
}

std::string gnd_plane(const std::string& layer, double w, double h) {
  return "  (zone (net 3) (net_name \"GND\") (layer \"" + layer + "\") (uuid \"g" + layer + "\") (hatch edge 0.5) (connect_pads (clearance 0.2))\n"
         "    (min_thickness 0.2) (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5) (island_removal_mode 1))\n"
         "    (polygon (pts (xy 0.5 0.5) (xy " + mm(w - 0.5) + " 0.5) (xy " + mm(w - 0.5) + " " + mm(h - 0.5) + ") (xy 0.5 " + mm(h - 0.5) + "))))\n";
}

// Six layers F.Cu In1 In2 In3 In4 B.Cu (0..5), `w` x `h` mm. Nets: S (1), B (2), GND (3), SD (4, class SD50), C (5), USB_P (6) and
// USB_N (7) in class USB90, VCC (8, class PWR_1A), HV (9, class HV_31_50).
std::string board(const std::string& extra, double w = 20, double h = 10) {
  return "(kicad_pcb (version 20240108) (generator \"pcbnew\")\n"
         "  (layers (0 \"F.Cu\" signal) (1 \"In1.Cu\" signal) (2 \"In2.Cu\" signal) (3 \"In3.Cu\" signal) (4 \"In4.Cu\" signal)\n"
         "    (31 \"B.Cu\" signal) (37 \"F.SilkS\" user) (44 \"Edge.Cuts\" user))\n"
         "  (net 0 \"\") (net 1 \"S\") (net 2 \"B\") (net 3 \"GND\") (net 4 \"SD\") (net 5 \"C\") (net 6 \"USB_P\") (net 7 \"USB_N\") (net 8 \"VCC\") (net 9 \"HV\")\n"
         "  (gr_rect (start 0 0) (end " + mm(w) + " " + mm(h) + ") (layer \"Edge.Cuts\") (stroke (width 0.1) (type solid)))\n" +
         extra + ")\n";
}

// The two JLC HDI rules of sensor_ts kicad/sensor/sensor.kicad_dru (lines 19 and 20), verbatim.
const std::string kJlcRules =
    "(rule \"jlc-hdi-microvia-hole-to-hole\" (condition \"A.Via_Type == 'Micro' || B.Via_Type == 'Micro'\") (constraint hole_to_hole (min 0.24mm)))\n"
    "(rule \"jlc-hdi-microvia-edge\" (condition \"A.Via_Type == 'Micro'\") (constraint edge_clearance (min 0.35mm)))\n";
// sensor.kicad_dru lines 16 and 17, verbatim.
const std::string kInnerWidths =
    "(rule \"sd50-inner\" (condition \"A.NetClass == 'SD50' && A.Type == 'Track'\") (layer inner) (constraint track_width (min 0.13mm) (opt 0.134mm) (max 0.14mm)))\n"
    "(rule \"usb90-inner\" (condition \"A.NetClass == 'USB90' && A.Type == 'Track'\") (layer inner) (constraint track_width (min 0.10mm) (opt 0.106mm) (max 0.112mm)) (constraint diff_pair_gap (min 0.15mm) (opt 0.152mm) (max 0.16mm)))\n";

// The project: JLC's HDI numbers, without KiCad 6's allow_microvias key (KiCad 10 has none: the sensor's project).
struct Files {
  fs::path dir, pcb;
  Files(const std::string& name, const std::string& text, const std::string& rules = "", double clearance = 0.1, double hole_clearance = 0.1) {
    dir = fs::temp_directory_path() / ("tmk_stacked_" + name + "_" + std::to_string(::getpid()));
    fs::remove_all(dir);
    fs::create_directories(dir);
    pcb = dir / "b.kicad_pcb";
    std::ofstream(pcb) << text;
    if (!rules.empty()) std::ofstream(dir / "b.kicad_dru") << "(version 1)\n" << rules;
    std::ofstream(dir / "b.kicad_pro")
        << R"({"board":{"design_settings":{"rules":{"min_clearance":)" << clearance << R"(,"min_track_width":0.1,"min_hole_clearance":)" << hole_clearance
        << R"(,"min_hole_to_hole":0.2,"min_copper_edge_clearance":0.2,"min_microvia_diameter":0.3,"min_microvia_drill":0.15,)"
        << R"("min_via_diameter":0.45,"min_through_hole_diameter":0.3,"min_via_annular_width":0.05}}},)"
        << R"("net_settings":{"classes":[{"name":"Default","clearance":)" << clearance
        << R"(,"track_width":0.1,"via_diameter":0.45,"via_drill":0.3,"microvia_diameter":0.3,"microvia_drill":0.15},)"
        << R"({"name":"SD50","clearance":)" << clearance << R"(,"track_width":0.174,"via_diameter":0.45,"via_drill":0.3,"microvia_diameter":0.3,"microvia_drill":0.15},)"
        << R"({"name":"USB90","clearance":0.152,"track_width":0.152,"diff_pair_width":0.152,"diff_pair_gap":0.152,"via_diameter":0.45,"via_drill":0.3,)"
        << R"("microvia_diameter":0.3,"microvia_drill":0.15},)"
        << R"({"name":"PWR_1A","clearance":)" << clearance << R"(,"track_width":0.4,"via_diameter":0.45,"via_drill":0.3,"microvia_diameter":0.3,"microvia_drill":0.15},)"
        << R"({"name":"HV_31_50","clearance":)" << clearance << R"(,"track_width":0.1,"via_diameter":0.45,"via_drill":0.3,"microvia_diameter":0.3,"microvia_drill":0.15}],)"
        << R"("netclass_patterns":[{"pattern":"SD","netclass":"SD50"},{"pattern":"USB_*","netclass":"USB90"},{"pattern":"VCC","netclass":"PWR_1A"},)"
        << R"({"pattern":"HV","netclass":"HV_31_50"}]}})";
  }
  ~Files() { fs::remove_all(dir); }
};

struct Loaded {
  io::LoadedBoard lb;
  model::DesignRules rules;
};
Loaded load(const Files& f) { return {io::read_board_file(f.pcb.string()), io::read_design_rules(f.pcb.string())}; }

route::RouterOptions options() {
  route::RouterOptions o;
  o.work_budget = 2'000'000;
  o.gpu_device = -1;
  o.time_limit_s = 600;
  o.optimize = false;
  o.seed = 1;
  o.stacked_vias = true;
  return o;
}

model::LayerMask bits(std::initializer_list<int> ls) {
  model::LayerMask m = 0;
  for (int l : ls) m |= model::layer_bit(l);
  return m;
}

// Net S from a pad at (3, 5) to one at (17, 5) on `side`, a keep-out across the middle of that side: S must go in and come back.
std::string crossing(const std::string& side, int net = 1, const std::string& name = "S") {
  return pad_fp("P1", 3, 5, side, net, name) + pad_fp("P2", 17, 5, side, net, name) + area("wall", side, 8, 0, 12, 10, true, true);
}

int count(const drc::DrcReport& r, const std::string& type) {
  int n = 0;
  for (const auto& v : r.violations) n += v.type == type;
  return n;
}

double dist(geom::Point a, geom::Point b) { return std::hypot(static_cast<double>(a.x - b.x), static_cast<double>(a.y - b.y)); }

// The board with the route's copper added, as the router's caller writes it.
model::Board with_route(const model::Board& b, const route::RouteResult& r) {
  model::Board out = b;
  for (const auto& t : r.tracks) out.tracks.push_back(t);
  for (const auto& v : r.vias) out.vias.push_back(v);
  return out;
}

// The micro vias of a result, grouped by position: each stack is one entry with its spans.
std::map<std::pair<Coord, Coord>, std::vector<std::pair<int, int>>> stacks_of(const route::RouteResult& r) {
  std::map<std::pair<Coord, Coord>, std::vector<std::pair<int, int>>> out;
  for (const auto& v : r.vias)
    if (v.type == model::ViaType::Micro) out[{v.pos.x, v.pos.y}].emplace_back(v.layer_top, v.layer_bottom);
  for (auto& [p, spans] : out) std::sort(spans.begin(), spans.end());
  return out;
}

}  // namespace

TEST_CASE("stacked vias: a layer change F.Cu to In2 is two micro vias at one point", "[route][stacked]") {
  const Files f("top", board(crossing("F.Cu")));
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 3, 4, 5});  // F.Cu and In2 only
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(r.routed == r.connections);
  const auto st = stacks_of(r);
  REQUIRE(st.size() == 2);
  for (const auto& [p, spans] : st) CHECK(spans == std::vector<std::pair<int, int>>{{0, 1}, {1, 2}});
  for (const auto& v : r.vias) {
    CHECK(v.type == model::ViaType::Micro);  // no through via where a stack goes
    CHECK(v.net == 1);
    CHECK(v.size == 300'000);
    CHECK(v.drill == 150'000);
    CHECK(v.filled);
  }
  CHECK(r.stacked_vias == 2);
  for (const auto& t : r.tracks) CHECK((t.layer == 0 || t.layer == 2));
  // Without --stacked-vias the same change is a through via, as before.
  o.stacked_vias = false;
  const auto plain = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(plain.routed == plain.connections);
  for (const auto& v : plain.vias) CHECK(v.type == model::ViaType::Through);
  CHECK(plain.stacked_vias == 0);
}

TEST_CASE("stacked vias: a layer change B.Cu to In3 is two micro vias at one point", "[route][stacked]") {
  const Files f("bottom", board(crossing("B.Cu")));
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({0, 1, 2, 4});  // In3 and B.Cu only
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(r.routed == r.connections);
  const auto st = stacks_of(r);
  REQUIRE(st.size() == 2);
  for (const auto& [p, spans] : st) CHECK(spans == std::vector<std::pair<int, int>>{{3, 4}, {4, 5}});
  CHECK(r.vias.size() == 4);
  for (const auto& t : r.tracks) CHECK((t.layer == 3 || t.layer == 5));
}

TEST_CASE("stacked vias: the late reachability proof (D154) takes a stack as a layer change", "[route][stacked]") {
  // Through vias are kept off the whole board (a via keep-out on B.Cu), so only a stack takes S from F.Cu to In2. The proof's
  // floods must make that move as the A* does, or the strict search is stopped as boxed in although a path exists.
  const Files f("late", board(crossing("F.Cu") + area("novia", "B.Cu", 0, 0, 20, 10, false, true)));
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 3, 4, 5});
  o.reach_check = 0;      // no pre-check, so the late proof runs
  o.reach_after = 1;      // in every strict search
  o.reach_after_cap = 0;  // and floods to the end
  o.rip_up = false;
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  CHECK(r.late_proofs > 0);  // the proof ran
  REQUIRE(r.routed == r.connections);
  CHECK(r.enclosed == 0);
  CHECK(stacks_of(r).size() == 2);
  for (const auto& v : r.vias) CHECK(v.type == model::ViaType::Micro);
}

TEST_CASE("stacked vias: the stack takes its In1 antipad from the refill and the router puts nothing else on In1", "[route][stacked]") {
  const Files f("antipad", board(crossing("F.Cu") + gnd_plane("In1.Cu", 20, 10)), kJlcRules);
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 3, 4, 5});
  o.soft_zones = true;  // as the farm routes: the plane is refilled round the new copper
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(r.routed == r.connections);
  const auto st = stacks_of(r);
  REQUIRE(st.size() == 2);
  for (const auto& t : r.tracks) CHECK(t.layer != 1);
  const auto rf = drc::refill_zones(with_route(l.lb.board, r), l.rules);
  // The In1 fill keeps 0.2 mm (the zone's clearance) from each stack's 0.30 mm pad: nothing within 0.35 mm, copper at 0.5 mm.
  const auto& z = *std::find_if(rf.board.zones.begin(), rf.board.zones.end(), [](const model::Zone& q) { return q.net == 3; });
  for (const auto& [p, spans] : st) {
    const geom::Point c{p.first, p.second};
    for (const auto& [layer, pts] : z.fills) {
      REQUIRE(layer == 1);
      for (const auto& q : pts) CHECK(dist(q, c) >= 0.35 * MM - 2'000);
    }
  }
  const auto rep = drc::run_drc(rf.board, l.rules);
  CHECK(count(rep, "clearance") == 0);
  CHECK(count(rep, "hole_clearance") == 0);
  CHECK(count(rep, "hole_to_hole") == 0);
  CHECK(count(rep, "copper_edge_clearance") == 0);
}

TEST_CASE("stacked vias: laser holes keep JLC's spacing, which KiCad does not check", "[route][stacked]") {
  // Copper clearance 0.05 mm, so that the hole rules decide. Net B has a stack at (10, 5) and a single F.Cu-In1 laser via at (5, 5).
  const std::string fixed =
      "  (via micro (at 10 5) (size 0.3) (drill 0.15) (layers \"F.Cu\" \"In1.Cu\") (net 2))\n"
      "  (via micro (at 10 5) (size 0.3) (drill 0.15) (layers \"In1.Cu\" \"In2.Cu\") (net 2))\n"
      "  (via micro (at 5 5) (size 0.3) (drill 0.15) (layers \"F.Cu\" \"In1.Cu\") (net 2))\n" +
      gnd_plane("In1.Cu", 20, 10);
  const Files f("laser_pairs", board(fixed), kJlcRules, 0.05, 0.05);
  auto l = load(f);
  route::Obstacles obs(l.lb.board, l.rules);
  obs.set_soft_zones(true);
  auto laser = [&](double x, int net, int l0, int l1) {
    return obs.via_state_span({static_cast<Coord>(x * MM), 5 * MM}, 300'000, 150'000, net, 0, false, nullptr, l0, l1, model::ViaType::Micro);
  };
  // Other nets, one laser layer pair: 0.24 mm edge to edge (the rule), 0.38 mm apart is 0.23 mm, 0.40 mm is 0.25 mm.
  CHECK(laser(10.38, 1, 0, 1) == 2);
  CHECK(laser(10.40, 1, 0, 1) == 0);
  // Two laser steps (F.Cu-In1 beside In1-In2) are no pair, as KiCad and the farm's micro_hole_spacing see them.
  CHECK(laser(5.38, 1, 1, 2) == 0);
  // The second half of a stack on the first one's spot: one net, two steps, no co-located holes.
  CHECK(laser(5.0, 2, 1, 2) == 0);
  CHECK(laser(5.0, 1, 1, 2) == 2);  // another net's copper is there
  // Plane voids: a stack keeps 0.95 mm from another net's laser via on its plane layer, 1.0 mm from another net's through via.
  obs.set_void_pitch(950'000, 1'000'000);
  CHECK(laser(10.94, 1, 0, 1) == 2);
  CHECK(laser(10.94, 1, 1, 2) == 2);
  CHECK(laser(10.96, 1, 0, 1) == 0);
  CHECK(laser(10.5, 2, 0, 1) == 0);  // the same net may come closer
  auto through = [&](double x, int net) { return obs.via_state({static_cast<Coord>(x * MM), 5 * MM}, 450'000, 300'000, net, 0, false); };
  CHECK(through(10.99, 1) == 2);
  CHECK(through(11.01, 1) == 0);
  CHECK(through(10.6, 3) == 0);  // GND has the plane on In1: its via leaves no void there
}

TEST_CASE("stacked vias: the plane-void pitches are plan 1.10's, and drops on different planes are no pair", "[route][stacked]") {
  // Net B: a top stack (F.Cu-In1-In2) at (10, 5) and a through via at (14, 5); GND planes on In1 and In4. Copper clearance 0.05 mm.
  const std::string fixed =
      "  (via micro (at 10 5) (size 0.3) (drill 0.15) (layers \"F.Cu\" \"In1.Cu\") (net 2))\n"
      "  (via micro (at 10 5) (size 0.3) (drill 0.15) (layers \"In1.Cu\" \"In2.Cu\") (net 2))\n"
      "  (via (at 14 5) (size 0.45) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 2))\n" +
      gnd_plane("In1.Cu", 20, 10) + gnd_plane("In4.Cu", 20, 10);
  const Files f("void_exact", board(fixed), kJlcRules, 0.05, 0.05);
  auto l = load(f);
  route::Obstacles obs(l.lb.board, l.rules);
  obs.set_soft_zones(true);
  obs.set_void_pitch(950'000, 1'000'000);
  auto laser = [&](Coord x, int l0, int l1) { return obs.via_state_span({x, 5 * MM}, 300'000, 150'000, 1, 0, false, nullptr, l0, l1, model::ViaType::Micro); };
  // Other-net drops on one plane: at least 0.95 mm centre to centre (0.95 itself is fine).
  CHECK(laser(10'949'999, 0, 1) == 2);
  CHECK(laser(10'950'000, 0, 1) == 0);
  // A drop and another net's through via: at least 1.0 mm.
  CHECK(laser(13'000'001, 0, 1) == 2);
  CHECK(laser(13'000'000, 0, 1) == 0);
  // A bottom drop (B.Cu-In4-In3, its void in In4) beside the top stack (its void in In1): no shared plane, no pitch; they share no
  // layer either, so no hole-to-hole pair (KiCad's rule binds holes that share a layer). The same spot is legal.
  CHECK(laser(10'500'000, 4, 5) == 0);
  CHECK(laser(10'500'000, 3, 4) == 0);
  CHECK(laser(10'000'000, 3, 4) == 0);
  CHECK(laser(10'500'000, 0, 1) == 2);  // a top drop there shares In1
}

TEST_CASE("stacked vias: the router keeps its stacks off other nets' vias by the plane-void pitch", "[route][stacked]") {
  // S may change layer only in a pocket beside each pad (keep-outs cover the rest of F.Cu: y 4.2 to 5.8 is open for x below 4.6
  // and above 15.4). Net B has a laser via (F.Cu-In1) in the west pocket and a through via in the east one, about 0.7 and 0.55 mm
  // from where S drops without the pitch (at the edge of its pad, and where the path meets the east pocket).
  const std::string extra = pad_fp("P1", 3, 5, "F.Cu", 1, "S") + pad_fp("P2", 17, 5, "F.Cu", 1, "S") + area("wall", "F.Cu", 4.6, 0, 15.4, 10, true, true) +
                            area("nw", "F.Cu", 0, 0, 4.6, 4.2, true, true) + area("sw", "F.Cu", 0, 5.8, 4.6, 10, true, true) +
                            area("ne", "F.Cu", 15.4, 0, 20, 4.2, true, true) + area("se", "F.Cu", 15.4, 5.8, 20, 10, true, true) +
                            "  (via micro (at 3.6 5.65) (size 0.3) (drill 0.15) (layers \"F.Cu\" \"In1.Cu\") (net 2))\n"
                            "  (via (at 15.55 5.55) (size 0.45) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 2))\n";
  const Files g("void_route", board(extra), kJlcRules);
  const auto lg = load(g);
  auto o = options();
  o.no_track_layers = bits({1, 3, 4, 5});
  auto nearest = [&](const route::RouteResult& r, model::ViaType type) {
    double d = 1e18;
    for (const auto& v : r.vias)
      for (const auto& q : lg.lb.board.vias)
        if (q.type == type) d = std::min(d, dist(v.pos, q.pos));
    return d;
  };
  const auto free_run = route::Router(lg.lb.board, lg.rules, o).run();
  REQUIRE(free_run.routed == free_run.connections);
  // Without the pitch the stacks go where the path meets the pockets' edges, beside B's vias.
  CHECK(nearest(free_run, model::ViaType::Micro) < 0.95 * MM);
  CHECK(nearest(free_run, model::ViaType::Through) < 1.0 * MM);
  o.stack_pitch = 950'000;
  o.stack_through_pitch = 1'000'000;
  const auto r = route::Router(lg.lb.board, lg.rules, o).run();
  REQUIRE(r.routed == r.connections);
  CHECK(r.stacked_vias == 2);
  CHECK(nearest(r, model::ViaType::Micro) >= 0.95 * MM);
  CHECK(nearest(r, model::ViaType::Through) >= 1.0 * MM);
}

TEST_CASE("stacked vias: a laser via keeps 0.24 mm from a drilled hole and 0.35 mm from the edge", "[route][stacked][rules]") {
  // A plated 0.3 mm hole (0.4 mm pad) of net B at (10, 5); copper clearance 0.05 mm and hole clearance 0.05 mm, so the hole-to-hole rule decides.
  const std::string pth =
      "  (footprint \"H\" (layer \"F.Cu\") (at 10 5) (property \"Reference\" \"H1\" (at 0 0) (layer \"F.SilkS\"))\n"
      "    (pad \"1\" thru_hole circle (at 0 0) (size 0.4 0.4) (drill 0.3) (layers \"*.Cu\") (net 2 \"B\")))\n";
  const Files f("drilled", board(pth), kJlcRules, 0.05, 0.05);
  auto l = load(f);
  route::Obstacles obs(l.lb.board, l.rules);
  CHECK(obs.rules().warnings().empty());
  auto laser = [&](double x, double y) {
    return obs.via_state_span({static_cast<Coord>(x * MM), static_cast<Coord>(y * MM)}, 300'000, 150'000, 1, 0, false, nullptr, 0, 1, model::ViaType::Micro);
  };
  // Hole edges 0.22 mm apart (centres 0.445 mm): the rule's 0.24 mm, not the board's 0.2 mm, applies to the probe.
  CHECK(laser(10.445, 5) == 2);
  CHECK(laser(10.47, 5) == 0);
  // The edge: 0.30 mm of air between the laser via's copper and the outline is under the rule's 0.35 mm, a through via there is fine.
  CHECK(laser(0.45, 5) == 2);
  CHECK(laser(0.51, 5) == 0);
  CHECK(obs.via_state({525'000, 5 * MM}, 450'000, 300'000, 1, 0, false) == 0);
  // The DRC agrees.
  const std::string vias =
      "  (via micro (at 10.445 5) (size 0.3) (drill 0.15) (layers \"F.Cu\" \"In1.Cu\") (net 1))\n"
      "  (via micro (at 0.45 8) (size 0.3) (drill 0.15) (layers \"F.Cu\" \"In1.Cu\") (net 1))\n"
      "  (via micro (at 15 5) (size 0.25) (drill 0.1) (layers \"F.Cu\" \"In1.Cu\") (net 1))\n";
  const Files g("drilled_drc", board(pth + vias), kJlcRules, 0.05, 0.05);
  const auto lg = load(g);
  const auto rep = drc::run_drc(lg.lb.board, lg.rules);
  CHECK(count(rep, "hole_to_hole") == 1);
  CHECK(count(rep, "copper_edge_clearance") == 1);
  // A micro via is held to the micro-via minimums (0.30 / 0.15 mm), not the via ones (0.45 / 0.30 mm), as KiCad's implicit
  // micro-via rule does: only the 0.25 / 0.10 mm one is too small.
  CHECK(count(rep, "via_diameter") == 1);
  CHECK(count(rep, "microvia_drill_out_of_range") == 1);  // KiCad's name for a micro via's
  CHECK(count(rep, "drill_out_of_range") == 0);
}

TEST_CASE("stacked vias: a router probe's via type reaches rule conditions", "[route][stacked][rules]") {
  // Bosch keep-out: no laser via on F.Cu inside it (a positional rule on the probe's type); and no laser via for net B anywhere.
  const std::string rules =
      "(rule \"bosch\" (layer F.Cu) (condition \"A.Via_Type == 'Micro' && A.intersectsArea('bosch')\") (constraint disallow via))\n"
      "(rule \"b-no-laser\" (condition \"A.Via_Type == 'Micro' && A.NetName == 'B'\") (constraint disallow via))\n";
  const Files f("probe_type", board(area("bosch", "F.Cu", 4, 3, 6, 7, false, false)), rules);
  auto l = load(f);
  route::Obstacles obs(l.lb.board, l.rules);
  const geom::Point in{5 * MM, 5 * MM}, out{15 * MM, 5 * MM};
  CHECK(obs.via_disallowed(in, 300'000, 1, 0, 1, model::ViaType::Micro));
  CHECK_FALSE(obs.via_disallowed(in, 450'000, 1, 0, 5, model::ViaType::Through));
  CHECK_FALSE(obs.via_disallowed(out, 300'000, 1, 0, 1, model::ViaType::Micro));
  CHECK(obs.via_state_span(in, 300'000, 150'000, 1, 0, false, nullptr, 0, 1, model::ViaType::Micro) == 2);
  CHECK_FALSE(obs.rules().via_allowed(2, model::ViaType::Micro));
  CHECK(obs.rules().via_allowed(2, model::ViaType::Through));
  CHECK(obs.rules().via_allowed(1, model::ViaType::Micro));
}

TEST_CASE("stacked vias: no stack where a rule or keep-out forbids a micro via", "[route][stacked][rules]") {
  // A net-wide `disallow micro_via`: S changes layer on through vias.
  const Files f("no_micro", board(crossing("F.Cu")), "(rule \"s-no-laser\" (condition \"A.NetName == 'S'\") (constraint disallow micro_via))\n");
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 3, 4, 5});
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(r.routed == r.connections);
  CHECK(r.stacked_vias == 0);
  for (const auto& v : r.vias) CHECK(v.type == model::ViaType::Through);
  // rf_X on In4: a via keep-out there; the bottom stacks (B.Cu-In4-In3) stay out of it.
  const Files g("rf_x", board(crossing("B.Cu") + area("rf_A", "In4.Cu", 3.5, 0, 8, 10, false, true)));
  auto lg = load(g);
  route::Obstacles obs(lg.lb.board, lg.rules);
  CHECK(obs.via_state_span({6 * MM, 5 * MM}, 300'000, 150'000, 1, 0, false, nullptr, 4, 5, model::ViaType::Micro) == 2);
  CHECK(obs.via_state_span({6 * MM, 5 * MM}, 300'000, 150'000, 1, 0, false, nullptr, 3, 4, model::ViaType::Micro) == 2);
  o.no_track_layers = bits({0, 1, 2, 4});
  const auto rg = route::Router(lg.lb.board, lg.rules, o).run();
  REQUIRE(rg.routed == rg.connections);
  REQUIRE(rg.stacked_vias == 2);
  for (const auto& v : rg.vias) CHECK((v.pos.x < 3'350'000 || v.pos.x > 8'150'000));
}

TEST_CASE("stacked vias: --split-cost sees the antipad of a stack the route adds", "[route][stacked][critical]") {
  // S is taken over from a routed board (a warm start): F.Cu to a stack at V (6.99, 9.99), In2 to a stack at W, F.Cu on. The plane
  // fills in the file know nothing of them. C, critical, runs straight past V at 0.33 mm, inside V's In1 antipad (0.15 + 0.2 mm).
  const double vx = 6.99, wx = 13.02, y = 9.99, cx = vx + 0.33;
  const Files f("split", board(pad_fp("P1", 3, y, "F.Cu", 1, "S", 0.4) + pad_fp("P2", 17.01, y, "F.Cu", 1, "S", 0.4) +
                               pad_fp("P3", cx, 2, "F.Cu", 5, "C", 0.4) + pad_fp("P4", cx, 18, "F.Cu", 5, "C", 0.4) + gnd_plane("In1.Cu", 20, 20),
                           20, 20));
  const auto l = load(f);
  auto o = options();
  o.stacked_vias = false;
  o.no_track_layers = bits({1, 2, 3, 4, 5});  // C has F.Cu only
  o.critical = {{"C", 1}};
  o.split_hard = true;
  auto at = [](double x, double yy) { return geom::Point{static_cast<Coord>(std::llround(x * MM)), static_cast<Coord>(std::llround(yy * MM))}; };
  o.warm_tracks = {model::Track{at(3, y), at(vx, y), 100'000, 0, 1, false, sexpr::kNoNode}, model::Track{at(vx, y), at(wx, y), 100'000, 2, 1, false, sexpr::kNoNode},
                   model::Track{at(wx, y), at(17.01, y), 100'000, 0, 1, false, sexpr::kNoNode}};
  for (const double x : {vx, wx})
    for (const int top : {0, 1}) o.warm_vias.push_back(model::Via{at(x, y), 300'000, 150'000, top, top + 1, model::ViaType::Micro, 1, false, sexpr::kNoNode});
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(r.routed == r.connections);
  // C keeps its centreline off V's antipad.
  double nearest = 1e18;
  for (const auto& t : r.tracks)
    if (t.net == 5) nearest = std::min(nearest, static_cast<double>(geom::point_seg_dist(at(vx, y), t.a, t.b)));
  CHECK(nearest >= 0.35 * MM);
}

TEST_CASE("stacked vias: rule widths per layer: SD50 runs at its inner opt on In2", "[route][stacked][rules]") {
  const Files f("widths", board(crossing("F.Cu", 4, "SD")), kInnerWidths);
  const auto l = load(f);
  auto o = options();
  o.stacked_vias = false;
  o.no_track_layers = bits({1, 3, 4, 5});
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(r.routed == r.connections);
  int inner = 0;
  for (const auto& t : r.tracks) {
    CHECK(t.width == (t.layer == 2 ? 134'000 : 174'000));
    inner += t.layer == 2;
  }
  CHECK(inner > 0);
  // KiCad's track_width check finds nothing.
  const auto rep = drc::run_drc(with_route(l.lb.board, r), l.rules);
  CHECK(count(rep, "track_width") == 0);
}

TEST_CASE("stacked vias: a USB90 pair stays off the layers where usb90-inner's gap binds it and it cannot couple", "[route][stacked][rules]") {
  // The pair must cross keep-outs on F.Cu and B.Cu, so only In2 and In3 lead across. usb90-inner asks for 0.106 mm and a
  // 0.15-0.16 mm gap there, which the coupled search (0.152 mm) cannot run and single halves cannot hold: the pair stays unrouted
  // rather than run uncoupled, with pair routing on or off. Without the gap constraint (the control) the halves cross on them.
  const std::string usb = pad_fp("U1", 3, 4.7, "F.Cu", 6, "USB_P", 0.3) + pad_fp("U2", 3, 5.3, "F.Cu", 7, "USB_N", 0.3) +
                          pad_fp("U3", 17, 4.7, "F.Cu", 6, "USB_P", 0.3) + pad_fp("U4", 17, 5.3, "F.Cu", 7, "USB_N", 0.3) +
                          area("wall", "F.Cu", 8, 0, 12, 10, true, true) + area("wall_b", "B.Cu", 8, 0, 12, 10, true, true);
  const Files f("usb_inner", board(usb), kInnerWidths);
  const auto l = load(f);
  auto o = options();
  o.stacked_vias = false;
  o.no_track_layers = bits({1, 4});  // the planes
  for (const bool pairs : {false, true}) {
    o.diff_pairs = pairs;
    const auto r = route::Router(l.lb.board, l.rules, o).run();
    CHECK(r.routed == 0);
    for (const auto& t : r.tracks) CHECK((t.layer == 0 || t.layer == 5));
  }
  const std::string width_only = kInnerWidths.substr(0, kInnerWidths.find("(rule \"usb90-inner\"")) +
                                 "(rule \"usb90-inner\" (condition \"A.NetClass == 'USB90' && A.Type == 'Track'\") (layer inner) "
                                 "(constraint track_width (min 0.10mm) (opt 0.106mm) (max 0.112mm)))\n";
  const Files g("usb_inner_free", board(usb), width_only);
  const auto lg = load(g);
  o.diff_pairs = false;
  const auto rg = route::Router(lg.lb.board, lg.rules, o).run();
  CHECK(rg.routed == rg.connections);
  CHECK(std::any_of(rg.tracks.begin(), rg.tracks.end(), [](const model::Track& t) { return t.layer == 2 || t.layer == 3; }));
}

TEST_CASE("stacked vias: --no-stack-classes: a rail changes layer on through vias, F.Cu-In2 included", "[route][stacked]") {
  const Files f("rail", board(crossing("F.Cu", 8, "VCC")));
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 3, 4, 5});  // F.Cu and In2 only
  o.no_stack_classes = {"PWR_*"};
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(r.routed == r.connections);
  CHECK(r.stacked_vias == 0);
  REQUIRE(r.vias.size() == 2);
  for (const auto& v : r.vias) {
    CHECK(v.type == model::ViaType::Through);
    CHECK(v.size == 450'000);
  }
  for (const auto& t : r.tracks) CHECK((t.layer == 0 || t.layer == 2));
  // Without the option the rail stacks like any net.
  o.no_stack_classes.clear();
  const auto stacked = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(stacked.routed == stacked.connections);
  CHECK(stacked.stacked_vias == 2);
}

TEST_CASE("stacked vias: the routed board connects, writes and reads back with no new DRC violation", "[route][stacked]") {
  const Files f("roundtrip", board(crossing("F.Cu") + crossing("B.Cu", 2, "B") + gnd_plane("In1.Cu", 20, 10) + gnd_plane("In4.Cu", 20, 10)), kJlcRules);
  auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 4});
  o.soft_zones = true;
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(r.routed == r.connections);
  CHECK(r.stacked_vias == 4);
  const auto before = drc::run_drc(drc::refill_zones(l.lb.board, l.rules).board, l.rules);
  io::BoardEditor ed(l.lb);
  for (const auto& t : r.tracks) ed.add_track(t);
  for (const auto& v : r.vias) ed.add_via(v);
  const std::string text = l.lb.doc.write();
  CHECK(text.find("(capping yes)") != std::string::npos);
  CHECK(text.find("(filling yes)") != std::string::npos);
  const fs::path out = f.dir / "out.kicad_pcb";
  std::ofstream(out) << text;
  fs::copy_file(f.dir / "b.kicad_pro", f.dir / "out.kicad_pro");
  fs::copy_file(f.dir / "b.kicad_dru", f.dir / "out.kicad_dru");
  const auto back = io::read_board_file(out.string());
  CHECK(back.board.vias.size() == r.vias.size());
  const auto after = drc::run_drc(drc::refill_zones(back.board, l.rules).board, l.rules);
  CHECK(after.unconnected.empty());
  CHECK_FALSE(before.unconnected.empty());
  auto b = before.counts(), a = after.counts();
  for (const auto& [type, n] : a) {
    INFO(type);
    CHECK(n <= b[type]);
  }
}

TEST_CASE("stacked vias: staggered, the second laser via sits 0.30 mm off with a stub on In1", "[route][stacked]") {
  const Files f("stagger", board(crossing("F.Cu") + gnd_plane("In1.Cu", 20, 10)), kJlcRules);
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 3, 4, 5});
  o.soft_zones = true;
  o.stagger = 300'000;
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(r.routed == r.connections);
  CHECK(r.stacked_vias == 2);
  std::vector<model::Via> upper, lower;
  for (const auto& v : r.vias) {
    REQUIRE(v.type == model::ViaType::Micro);
    (v.layer_top == 0 ? upper : lower).push_back(v);
  }
  REQUIRE(upper.size() == 2);
  REQUIRE(lower.size() == 2);
  int stubs = 0;
  for (const auto& u : upper) {
    const auto lo = std::min_element(lower.begin(), lower.end(), [&](const auto& a, const auto& c) { return dist(a.pos, u.pos) < dist(c.pos, u.pos); });
    CHECK(std::abs(dist(lo->pos, u.pos) - 0.30 * MM) <= 1'000);
    CHECK(lo->layer_top == 1);
    CHECK(lo->layer_bottom == 2);
    for (const auto& t : r.tracks)
      if (t.layer == 1 && ((t.a == u.pos && t.b == lo->pos) || (t.b == u.pos && t.a == lo->pos))) {
        CHECK(t.width == 300'000);
        ++stubs;
      }
  }
  CHECK(stubs == 2);
  for (const auto& t : r.tracks) CHECK((t.layer != 1 || t.width == 300'000));
  // The same rules as a stack: KiCad's checks hold on the refilled board, and connectivity joins the halves through the stub.
  const auto rep = drc::run_drc(drc::refill_zones(with_route(l.lb.board, r), l.rules).board, l.rules);
  CHECK(rep.unconnected.empty());
  CHECK(count(rep, "clearance") == 0);
  CHECK(count(rep, "hole_clearance") == 0);
  CHECK(count(rep, "hole_to_hole") == 0);
}

TEST_CASE("stacked vias: staggered stacks keep the width rules and the spacing rules of stacked ones", "[route][stacked]") {
  // An SD50 net with the sensor's sd50-inner rule: its In1 stub is a track on an inner layer, so it takes 0.134 mm, not the via's
  // 0.30 mm (KiCad's track_width would flag that).
  const Files f("stagger_sd", board(crossing("F.Cu", 4, "SD") + gnd_plane("In1.Cu", 20, 10)), kJlcRules + kInnerWidths);
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 3, 4, 5});
  o.soft_zones = true;
  o.stagger = 300'000;
  const auto r = route::Router(l.lb.board, l.rules, o).run();
  REQUIRE(r.routed == r.connections);
  CHECK(r.stacked_vias == 2);
  int stubs = 0;
  for (const auto& t : r.tracks)
    if (t.layer == 1) {
      CHECK(t.width == 134'000);
      ++stubs;
    }
  CHECK(stubs == 2);
  const auto rep = drc::run_drc(drc::refill_zones(with_route(l.lb.board, r), l.rules).board, l.rules);
  CHECK(count(rep, "track_width") == 0);
  CHECK(rep.unconnected.empty());
  // Net B has an In1-In2 laser via 0.7 mm east of where S's west drop goes (beside its pad, the inner track leaving east). With the
  // plane-void pitch the inner half keeps 0.95 mm from it as a stacked one would; without it (the control) it sits closer.
  const std::string other = crossing("F.Cu") + gnd_plane("In1.Cu", 20, 10) +
                            "  (via micro (at 4.3 5.01) (size 0.3) (drill 0.15) (layers \"In1.Cu\" \"In2.Cu\") (net 2))\n";
  const Files g("stagger_pitch", board(other), kJlcRules);
  const auto lg = load(g);
  auto nearest = [&](const route::RouteResult& rr) {
    double d = 1e18;
    for (const auto& v : rr.vias) d = std::min(d, dist(v.pos, geom::Point{4'300'000, 5'010'000}));
    return d;
  };
  const auto free_run = route::Router(lg.lb.board, lg.rules, o).run();
  REQUIRE(free_run.routed == free_run.connections);
  CHECK(nearest(free_run) < 0.95 * MM);
  o.stack_pitch = 950'000;
  o.stack_through_pitch = 1'000'000;
  const auto rp = route::Router(lg.lb.board, lg.rules, o).run();
  REQUIRE(rp.routed == rp.connections);
  CHECK(rp.stacked_vias == 2);
  CHECK(nearest(rp) >= 0.95 * MM);
  // and every laser hole of one layer pair keeps the 0.24 mm rule from it (In1-In2 halves: centres 0.39 mm apart at least).
  for (const auto& v : rp.vias)
    if (v.layer_top == 1) CHECK(dist(v.pos, geom::Point{4'300'000, 5'010'000}) >= 0.39 * MM);
}

namespace {

// TM-5: net `name` changes side, from a pad on F.Cu at (3, 5) to one on B.Cu at (17, 5), with tracks on F.Cu and B.Cu only, so on one
// through via. Via keep-outs leave two sites: window A on the straight line (6.5-7.5, 4.5-5.5) and window B off it (12.6-13.4,
// 6.4-7.0), 1.4 mm of octilinear track longer. A fixed GND through via sits at (13, gnd_y).
std::string two_sites(int net, const std::string& name, double gnd_y) {
  std::string s = pad_fp("P1", 3, 5, "F.Cu", net, name) + pad_fp("P2", 17, 5, "B.Cu", net, name) + "  (via (at 13 " + mm(gnd_y) +
                  ") (size 0.45) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 3))\n";
  const double k[7][4] = {{0, 0, 6.5, 10},   {7.5, 0, 12.6, 10},   {13.4, 0, 20, 10},   {6.5, 0, 7.5, 4.5},
                          {6.5, 5.5, 7.5, 10}, {12.6, 0, 13.4, 6.4}, {12.6, 7.0, 13.4, 10}};
  for (int i = 0; i < 7; ++i) s += area("nv" + std::to_string(i), "F.Cu", k[i][0], k[i][1], k[i][2], k[i][3], false, true);
  return s;
}

bool in_a(geom::Point p) { return p.x >= 6'500'000 && p.x <= 7'500'000 && p.y >= 4'500'000 && p.y <= 5'500'000; }
bool in_b(geom::Point p) { return p.x >= 12'600'000 && p.x <= 13'400'000 && p.y >= 6'400'000 && p.y <= 7'000'000; }

// The one through via of a two_sites route.
geom::Point side_via(const route::RouteResult& r) {
  REQUIRE(r.routed == r.connections);
  REQUIRE(r.vias.size() == 1);
  CHECK(r.vias[0].type == model::ViaType::Through);
  return r.vias[0].pos;
}

}  // namespace

TEST_CASE("return via cost: a side change takes the via site with a GND through via within 1.0 mm", "[route][return]") {
  const geom::Point gnd{13'000'000, 7'600'000};  // every site of window B is 0.83-0.99 mm from it
  const Files f("ret_site", board(two_sites(1, "S", 7.6)));
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 2, 3, 4});
  // Off (the default): site A on the straight line, with no return in reach, as before.
  const auto off = side_via(route::Router(l.lb.board, l.rules, o).run());
  CHECK(in_a(off));
  CHECK(dist(off, gnd) > 1.0 * MM);
  // On: site B with its GND return, for 1.4 mm more track (the penalty is one via's 3 mm).
  o.return_via_cost = 1;
  const auto on = side_via(route::Router(l.lb.board, l.rules, o).run());
  CHECK(in_b(on));
  CHECK(dist(on, gnd) <= 1.0 * MM);
  // An exempt net pays nothing: site A.
  o.return_via_exempt = {"S"};
  CHECK(in_a(side_via(route::Router(l.lb.board, l.rules, o).run())));
  // Nor does a rail (PWR_*, one of the farm's POWER_CLASSES, the default skip list).
  o.return_via_exempt.clear();
  const Files g("ret_rail", board(two_sites(8, "VCC", 7.6)));
  const auto lg = load(g);
  CHECK(in_a(side_via(route::Router(lg.lb.board, lg.rules, o).run())));
  // Without the skip list the rail takes site B like a signal.
  o.return_via_skip_classes.clear();
  CHECK(in_b(side_via(route::Router(lg.lb.board, lg.rules, o).run())));
}

TEST_CASE("return via cost: an HV_31_50 net's return reaches 1.25 mm (--return-via-radius-class)", "[route][return]") {
  const geom::Point gnd{13'000'000, 7'900'000};  // every site of window B is 1.12-1.29 mm from it
  const Files f("ret_hv", board(two_sites(9, "HV", 7.9)));
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 2, 3, 4});
  o.return_via_cost = 1;
  // At the default 1.0 mm neither site has a return: both pay, the shorter A wins.
  CHECK(in_a(side_via(route::Router(l.lb.board, l.rules, o).run())));
  // At the class's 1.25 mm site B has one.
  o.return_via_radius_class = {{"HV_31_50*", 1'250'000}};
  const auto hv = side_via(route::Router(l.lb.board, l.rules, o).run());
  CHECK(in_b(hv));
  CHECK(dist(hv, gnd) > 1.0 * MM);
  CHECK(dist(hv, gnd) <= 1.25 * MM);
  // The class entry does not reach a Default net: S keeps 1.0 mm and site A.
  const Files g("ret_hv_s", board(two_sites(1, "S", 7.9)));
  const auto lg = load(g);
  CHECK(in_a(side_via(route::Router(lg.lb.board, lg.rules, o).run())));
}

TEST_CASE("return via cost: the LNS clean-up does not trade a covered side change for an uncovered one", "[route][return]") {
  // Net B runs straight across window B at y = 6.7, so with S's via at B it detours; S is the shorter net and goes first.
  // Without S, the clean-up (lns_vias) routes B straight through the window and S last: it takes site A, 1.4 mm shorter,
  // and the group "improves" on length and vias alone. With the penalty the main pass chose the covered site B.
  const geom::Point gnd{13'000'000, 7'600'000};
  const Files f("ret_lns", board(two_sites(1, "S", 7.6) + pad_fp("N1", 1, 6.7, "F.Cu", 2, "B") + pad_fp("N2", 19, 6.7, "F.Cu", 2, "B")));
  const auto l = load(f);
  auto o = options();
  o.no_track_layers = bits({1, 2, 3, 4});
  o.optimize = true;
  auto via_of_s = [&](const route::RouteResult& r) {
    REQUIRE(r.routed == r.connections);
    geom::Point s{};
    int n = 0;
    for (const auto& v : r.vias)
      if (v.net == 1) {
        s = v.pos;
        ++n;
      }
    REQUIRE(n == 1);
    return s;
  };
  // Without the cost the straight site A, as in the first test.
  CHECK(in_a(via_of_s(route::Router(l.lb.board, l.rules, o).run())));
  // On, the main pass takes the covered site B; the clean-up must keep it.
  o.return_via_cost = 1;
  o.optimize = false;
  CHECK(in_b(via_of_s(route::Router(l.lb.board, l.rules, o).run())));
  o.optimize = true;
  const auto on = via_of_s(route::Router(l.lb.board, l.rules, o).run());
  CHECK(in_b(on));
  CHECK(dist(on, gnd) <= 1.0 * MM);
}
