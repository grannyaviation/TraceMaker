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
  void conn(const std::string& name, double ax, double ay, double bx, double by) {
    for (const auto& [id, x, y] : {std::tuple{name + "_a", ax, ay}, std::tuple{name + "_b", bx, by}})
      obstacles.push_back({{"type", "rect"}, {"layers", {"top"}}, {"center", {{"x", x}, {"y", y}}}, {"width", 0.5}, {"height", 0.5}, {"connectedTo", {id}}});
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

}  // namespace

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
