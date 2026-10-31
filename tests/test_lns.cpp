// SPDX-License-Identifier: GPL-3.0-or-later
// The completion LNS (--lns-complete, fork): synthetic cases of the ways it gave up on the GraNNy sensor board (TM-6).
#include <catch2/catch_test_macros.hpp>

#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

#include "drc/drc.hpp"
#include "io/kicad/board_reader.hpp"
#include "io/kicad/project_reader.hpp"
#include "io/srj/srj.hpp"
#include "route/router.hpp"

namespace fs = std::filesystem;
using nlohmann::json;
using namespace tmk;

namespace {

struct Loaded {
  model::Board board;
  model::DesignRules rules;
};

Loaded load(const json& problem, const std::string& tag) {
  const auto files = io::srj_to_kicad(io::read_srj(problem.dump()));
  const fs::path dir = fs::temp_directory_path() / ("tm_lns_test_" + tag + "_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  const fs::path pcb = dir / "b.kicad_pcb";
  std::ofstream(pcb) << files.pcb;
  std::ofstream(dir / "b.kicad_pro") << files.pro;
  if (!files.dru.empty()) std::ofstream(dir / "b.kicad_dru") << files.dru;
  Loaded l{io::read_board_file(pcb.string()).board, io::read_design_rules(pcb.string())};
  fs::remove_all(dir);
  return l;
}

// One copper layer, 30 x 20 mm, 0.15 mm tracks and clearance.
struct Problem {
  json obstacles = json::array(), connections = json::array();
  void keepout(double x0, double y0, double x1, double y1) {
    obstacles.push_back({{"type", "rect"}, {"layers", {"top"}}, {"center", {{"x", (x0 + x1) / 2}, {"y", (y0 + y1) / 2}}}, {"width", x1 - x0}, {"height", y1 - y0},
                         {"connectedTo", json::array()}});
  }
  void conn(const std::string& name, double ax, double ay, double bx, double by, double a_size = 0.5) {
    for (const auto& [id, x, y, s] : {std::tuple{name + "_a", ax, ay, a_size}, std::tuple{name + "_b", bx, by, 0.5}})
      obstacles.push_back({{"type", "rect"}, {"layers", {"top"}}, {"center", {{"x", x}, {"y", y}}}, {"width", s}, {"height", s}, {"connectedTo", {id}}});
    connections.push_back({{"name", name},
                           {"pointsToConnect",
                            {{{"x", ax}, {"y", ay}, {"layer", "top"}, {"pointId", name + "_a"}}, {{"x", bx}, {"y", by}, {"layer", "top"}, {"pointId", name + "_b"}}}},
                           {"rootConnectionName", name}});
  }
  json done() const {
    return {{"layerCount", 1},          {"minTraceWidth", 0.15},    {"defaultObstacleMargin", 0.15},
            {"bounds", {{"minX", 0.0}, {"maxX", 30.0}, {"minY", 0.0}, {"maxY", 20.0}}},
            {"obstacles", obstacles}, {"connections", connections}};
  }
};

route::RouterOptions lns_options() {
  route::RouterOptions o;
  o.lns_complete = true;
  o.bend_states = false;  // --fast-bends, as the farm
  o.work_budget = 20'000'000;
  o.time_limit_s = 3600;  // the work budget decides
  o.gpu_device = -1;
  return o;
}

int errors(model::Board b, const model::DesignRules& r, const route::RouteResult& res) {
  for (const auto& t : res.tracks) b.tracks.push_back(t);
  for (const auto& v : res.vias) b.vias.push_back(v);
  int n = 0;
  for (const auto& v : drc::run_drc(b, r).violations)
    if (v.severity == "error" && v.type != "unconnected_items") ++n;
  return n;
}

// A wall at x 14-16 with two one-track gaps: G2 at y 14, G3 at y 4. W (12, 15) -> (18, 15) is the shortest and takes G2 in the strict
// pass; V (11.5, 13.2) -> (18.5, 13.2) then finds G2 taken and G3 outside its strict windows. Both fit: V through G2, W through G3.
// The completion LNS lifts W for V; W's strict windows miss G3 too, and W's probe finds its way back across V's new path in its first
// (2 mm) window, so the two swap until the chain gives up: "W, lifted for it, finds its way back only across it".
json swap_problem() {
  Problem p;
  p.keepout(14, 0, 16, 3.7);
  p.keepout(14, 4.3, 16, 13.7);
  p.keepout(14, 14.3, 16, 20);
  p.conn("W", 12, 15, 18, 15);
  p.conn("V", 11.5, 13.2, 18.5, 13.2);
  return p.done();
}

// Two sections, one layer, 40 x 40 mm.
// Left: a wall at x 9-10 with one-track gaps at y 20 (G1), 25 (G2) and 6 (G3). X1 (7, 21) -> (12, 21) takes G1 and X2 (7, 25) ->
// (12, 25) takes G2; B (2, 20) -> (17, 20) then finds both taken and G3 outside its strict windows. B closes only when X1 moves to
// G2 and X2 to G3: two connections lifted, so with --lns-cap 1 not at level 0 (cap 1), only at level 1 (cap 2).
// Right: a wall at x 29-30 with gaps at y 20 (H1) and 27 (H2). W2 (27, 21) -> (32, 21) takes H1, V2 (26.5, 17) -> (32.5, 17) finds it
// taken and H2 outside its windows; one lift (W2 to H2) closes it in sweep 0. That gain keeps the LNS's level at 0 for sweep 1.
json escalate_problem() {
  Problem p;
  auto wall = [&](double x0, double x1, std::vector<double> gaps) {
    std::sort(gaps.begin(), gaps.end());
    double y = 0;
    for (double g : gaps) {
      p.keepout(x0, y, x1, g - 0.3);
      y = g + 0.3;
    }
    p.keepout(x0, y, x1, 40);
  };
  wall(9, 10, {20, 25, 6});
  wall(29, 30, {20, 27});
  p.conn("X1", 7, 21, 12, 21);
  p.conn("X2", 7, 25, 12, 25);
  p.conn("B", 2, 20, 17, 20);
  p.conn("W2", 27, 21, 32, 21);
  p.conn("V2", 26.5, 17, 32.5, 17);
  json j = p.done();
  j["bounds"] = {{"minX", 0.0}, {"maxX", 40.0}, {"minY", 0.0}, {"maxY", 40.0}};
  return j;
}

// One layer, 30 x 24 mm. A band of walls at y 9-11 splits the board; its only passage is a one-track channel at x 10.4-11.0. Q's pin
// (a 0.16 mm pad at (10, 10.025)) sits in a dead-end slot of the band whose mouth opens into that channel: the slot is 0.47 mm wide and
// centred between two lattice rows, so no lattice point in it is legal and Q leaves only by its off-lattice exit along the slot. Q goes
// up the channel, through gap G1 (y 13) of a wall at x 14-16, to (19, 15). U1 (9, 12.5) -> (9, 7.5) needs the channel, which walls Q
// in: it never routes. U2 (12.5, 13.6) -> (19.5, 13.6) needs G1; Q can give it way only through the far gap G2 (y 22), outside both
// strict windows, so Q comes back by its probe. U1 is the shorter, so its chain comes first in a sweep (with U2 first, U2's chain
// works out Q's exit without U1's copper and the case does not arise).
json exit_problem() {
  Problem p;
  const double c = 10.025;
  p.keepout(0, c + 0.235, 10.4, 11);  // the band left of the channel, with the slot cut into it
  p.keepout(0, 9, 10.4, c - 0.235);
  p.keepout(0, c - 0.235, 9.6, c + 0.235);
  p.keepout(11, 9, 30, 11);           // the band right of the channel
  p.keepout(14, 11, 16, 12.7);        // the wall with G1 and G2
  p.keepout(14, 13.3, 16, 21.7);
  p.keepout(14, 22.3, 16, 24);
  p.conn("Q", 10, c, 19, 15, 0.16);
  p.conn("U1", 9, 12.5, 9, 7.5);
  p.conn("U2", 12.5, 13.6, 19.5, 13.6);
  json j = p.done();
  j["bounds"] = {{"minX", 0.0}, {"maxX", 30.0}, {"minY", 0.0}, {"maxY", 24.0}};
  return j;
}

// The band of exit_problem() with two more one-track channels: C2 at x 6.4-7.0, beside the slot, and C3 at x 22.7-23.3, far off.
// Q (the slot pin) goes up C1 to (13, 13); R (6.7, 12) -> (20, 7) runs down C2. U1 (9, 12.5) -> (9, 7.5) reaches C3 in no strict
// window, so it needs C1 or C2. Its probe takes C1, the shorter crossing, and walls Q in; with Q's old copper to pay for, it takes
// C2 and lifts R, which comes back through C3.
json retry_problem() {
  Problem p;
  const double c = 10.025;
  p.keepout(0, 9, 6.4, 11);
  p.keepout(7.0, c + 0.235, 10.4, 11);
  p.keepout(7.0, 9, 10.4, c - 0.235);
  p.keepout(7.0, c - 0.235, 9.6, c + 0.235);
  p.keepout(11, 9, 22.7, 11);
  p.keepout(23.3, 9, 30, 11);
  p.conn("Q", 10, c, 13, 13, 0.16);
  p.conn("R", 6.7, 12, 20, 7);
  p.conn("U1", 9, 12.5, 9, 7.5);
  json j = p.done();
  j["bounds"] = {{"minX", 0.0}, {"maxX", 30.0}, {"minY", 0.0}, {"maxY", 24.0}};
  return j;
}

// `nets` on the board first, unsearched (as --warm-pcb puts them): routed alone, first, in one strict pass, then handed over as tracks.
void warm_start(const Loaded& l, const std::vector<std::string>& nets, route::RouterOptions& o) {
  route::RouterOptions first = lns_options();
  first.lns_complete = false;
  first.first_nets = nets;
  first.max_passes = 1;
  const auto r0 = route::route_portfolio(l.board, l.rules, first, 1, {}, 1).best;
  for (const auto& t : r0.tracks)
    if (std::find(nets.begin(), nets.end(), l.board.nets[static_cast<std::size_t>(t.net)].name) != nets.end()) o.warm_tracks.push_back(t);
}

}  // namespace

TEST_CASE("completion LNS: an exit that only a failed protected chain's copper blocked is open again after the chain", "[route][lns]") {
  const auto l = load(exit_problem(), "exit");
  // Q goes on the board unsearched, as --warm-pcb (or TM_LNS_RESUME) puts it, so the LNS is the first to work out its pin's exit.
  route::RouterOptions on = lns_options();
  warm_start(l, {"Q"}, on);
  REQUIRE(!on.warm_tracks.empty());
  on.lns_protect = true;
  on.lns_escalate = true;
  const auto r = route::route_portfolio(l.board, l.rules, on, 1, {}, 1).best;
  REQUIRE(r.connections == 3);
  // U1's chain walls Q in and rolls back; U2's chain, later in the same sweep, needs Q's exit again.
  CHECK(r.routed == 2);
  REQUIRE(r.failures.size() == 1);
  CHECK(r.failures[0].find("U1") != std::string::npos);
  CHECK(errors(l.board, l.rules, r) == errors(l.board, l.rules, {}));
}

TEST_CASE("completion LNS: a connection that needs the next level gets it in the next sweep, not after a sweep without gain", "[route][lns]") {
  const auto l = load(escalate_problem(), "escalate");
  route::RouterOptions plain = lns_options();
  plain.lns_cap = 1;
  plain.lns_protect = true;
  const auto a = route::route_portfolio(l.board, l.rules, plain, 1, {}, 1).best;
  REQUIRE(a.connections == 5);
  CHECK(a.routed == 5);
  CHECK(a.lns_last_gain_sweep == 2);  // sweep 0 gains V2, sweep 1 retries B at level 0, sweep 2 (level 1) gains B

  route::RouterOptions esc = plain;
  esc.lns_escalate = true;
  const auto b = route::route_portfolio(l.board, l.rules, esc, 1, {}, 1).best;
  CHECK(b.routed == 5);
  CHECK(b.lns_last_gain_sweep == 1);  // B failed at level 0 in sweep 0, so it is tried at level 1 in sweep 1
  CHECK(b.expansions < a.expansions);
  CHECK(errors(l.board, l.rules, b) == errors(l.board, l.rules, {}));
}

TEST_CASE("completion LNS: a lifted connection that can only come back across the open one ends in a swap; protected, it goes round", "[route][lns]") {
  const auto l = load(swap_problem(), "swap");
  route::RouterOptions off = lns_options();
  const auto a = route::route_portfolio(l.board, l.rules, off, 1, {}, 1).best;
  REQUIRE(a.connections == 2);
  CHECK(a.routed == 1);  // the cause, as on the board
  REQUIRE(a.failures.size() == 1);
  CHECK(a.failures[0].find("finds its way back only across it") != std::string::npos);

  route::RouterOptions on = lns_options();
  on.lns_protect = true;
  const auto b = route::route_portfolio(l.board, l.rules, on, 1, {}, 1).best;
  CHECK(b.routed == 2);
  CHECK(errors(l.board, l.rules, b) == errors(l.board, l.rules, {}));  // the walls touch the board edge: no error added
}

TEST_CASE("completion LNS, protected: a chain that walls a lifted connection in is tried again with u paying for that one's old copper", "[route][lns]") {
  const auto l = load(retry_problem(), "retry");
  route::RouterOptions on = lns_options();
  warm_start(l, {"Q", "R"}, on);
  REQUIRE(!on.warm_tracks.empty());
  on.lns_protect = true;
  const auto r = route::route_portfolio(l.board, l.rules, on, 1, {}, 1).best;
  REQUIRE(r.connections == 3);
  CHECK(r.routed == 3);
  CHECK(r.lns_last_gain_sweep == 0);  // the first chain walls Q in, its retry closes U1
  CHECK(errors(l.board, l.rules, r) == errors(l.board, l.rules, {}));

  // --lns-escalate: below level 2 only the first chain runs, so the retry that closes U1 comes in sweep 2.
  on.lns_escalate = true;
  const auto e = route::route_portfolio(l.board, l.rules, on, 1, {}, 1).best;
  CHECK(e.routed == 3);
  CHECK(e.lns_last_gain_sweep == 2);
}
