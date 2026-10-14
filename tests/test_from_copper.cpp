// SPDX-License-Identifier: GPL-3.0-or-later
// A connection continues from its net's existing copper: a pre-routed stub and via (a hand-routed or generated
// fan-out) are endpoints of the search, not only the pads.
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <string>

#include "io/kicad/board_reader.hpp"
#include "route/router.hpp"

using namespace tmk;

namespace {

// 30 x 20 mm, two layers. Net A joins F.Cu pads at x 5 and x 25. A locked wall of net W runs across F.Cu at x 12,
// and a rule area keeps vias out of x < 12: pad R1 can only leave F.Cu through the existing via at (9, 10) at the
// end of its pre-routed stub.
model::Board board() {
  return io::read_board(sexpr::Document::parse(R"((kicad_pcb (version 20240108) (generator "pcbnew")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "") (net 1 "A") (net 2 "W")
  (footprint "T:P" (layer "F.Cu") (at 5 10) (property "Reference" "R1")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "A")))
  (footprint "T:P" (layer "F.Cu") (at 25 10) (property "Reference" "R2")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "A")))
  (segment (start 5 10) (end 9 10) (width 0.25) (layer "F.Cu") (net 1))
  (via (at 9 10) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 1))
  (segment (start 12 0.5) (end 12 19.5) (width 0.5) (locked yes) (layer "F.Cu") (net 2))
  (zone (net 0) (net_name "") (layers "F.Cu" "B.Cu") (name "no vias") (hatch edge 0.5) (connect_pads (clearance 0))
    (min_thickness 0.25) (keepout (tracks allowed) (vias not_allowed) (pads allowed) (copperpour allowed) (footprints allowed))
    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5)) (polygon (pts (xy 0 0) (xy 12 0) (xy 12 20) (xy 0 20))))
  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
))"));
}

model::DesignRules rules() {
  model::DesignRules r;
  r.minimums.clearance = 150'000;
  r.minimums.track_width = 150'000;
  r.minimums.via_diameter = 600'000;
  r.minimums.through_hole_diameter = 300'000;
  r.minimums.hole_clearance = 200'000;
  model::NetClass d;
  d.name = "Default";
  d.clearance = 200'000;
  d.track_width = 250'000;
  d.via_diameter = 800'000;
  d.via_drill = 400'000;
  r.classes = {d};
  return r;
}

}  // namespace

TEST_CASE("from copper: a connection continues from the existing stub's via", "[route][from_copper]") {
  route::RouterOptions o;
  o.work_budget = 40'000'000;
  o.time_limit_s = 600;
  o.gpu_device = -1;
  const auto r = route::Router(board(), rules(), o).run();
  CHECK(r.connections == 1);
  CHECK(r.routed == 1);
  // The only way off F.Cu left of the wall is the existing via: every new via is right of it.
  CHECK(std::all_of(r.vias.begin(), r.vias.end(), [](const model::Via& v) { return v.pos.x > 12'000'000; }));
  // The new copper starts at the existing via on B.Cu: no new track on F.Cu left of the wall.
  CHECK(std::none_of(r.tracks.begin(), r.tracks.end(), [](const model::Track& t) {
    return t.layer == 0 && std::min(t.a.x, t.b.x) < 12'000'000;
  }));
}

TEST_CASE("from copper: a stub on the pad does not move the route off the pad centre", "[route][from_copper]") {
  // Pad R1 (2 x 2 mm at (5, 10)) carries a stub of its net from (5, 10.5) down to (5, 13). The lattice points of the
  // stub inside the pad were free starts (no leg to the pad), so the route left R1 from the stub, 0.5 mm off the
  // pad centre; on the pad the pad's own cells, with their leg from the centre, decide. (Starting on the stub
  // outside the pad is fine: that is existing copper.)
  const auto b = io::read_board(sexpr::Document::parse(R"((kicad_pcb (version 20240108) (generator "pcbnew")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "") (net 1 "A")
  (footprint "T:P" (layer "F.Cu") (at 5 10) (property "Reference" "R1")
    (pad "1" smd rect (at 0 0) (size 2 2) (layers "F.Cu") (net 1 "A")))
  (footprint "T:P" (layer "F.Cu") (at 25 10) (property "Reference" "R2")
    (pad "1" smd rect (at 0 0) (size 2 2) (layers "F.Cu") (net 1 "A")))
  (segment (start 5 10.5) (end 5 13) (width 0.2) (locked yes) (layer "F.Cu") (net 1))
  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
))"));
  route::RouterOptions o;
  o.work_budget = 40'000'000;
  o.time_limit_s = 600;
  o.gpu_device = -1;
  const auto r = route::Router(b, rules(), o).run();
  CHECK(r.routed == 1);
  const model::Point centre{5'000'000, 10'000'000};
  auto on_r1 = [](model::Point p) { return p.x >= 4'000'000 && p.x <= 6'000'000 && p.y >= 9'000'000 && p.y <= 11'000'000; };
  bool on_pad = false, from_centre = false;
  for (const auto& t : r.tracks) {
    UNSCOPED_INFO("track (" << t.a.x << "," << t.a.y << ")-(" << t.b.x << "," << t.b.y << ") L" << t.layer);
    on_pad = on_pad || on_r1(t.a) || on_r1(t.b);
    from_centre = from_centre || t.a == centre || t.b == centre;
  }
  CHECK((!on_pad || from_centre));  // a track that ends on R1's copper comes with the leg from its centre
}
