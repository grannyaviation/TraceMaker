// SPDX-License-Identifier: GPL-3.0-or-later
// The vias of one route keep the board's hole-to-hole distance between each other, not only to the copper already
// on the board. A wide net used to hop a narrow spot on two vias a lattice step or two apart (sensor_ts: V5V vias
// 0.275 mm apart with 0.3 mm drills, holes into each other), which fabs reject.
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>

#include "io/kicad/board_reader.hpp"
#include "route/router.hpp"

using namespace tmk;

namespace {

// 30 x 20 mm, four layers. Net A runs from an F.Cu pad at (5, 10) to a B.Cu pad at (25, 10) with 1.3 mm tracks and
// 0.6/0.3 mm vias. Locked walls of net W (0.2 mm, clearance 0.2): B.Cu at x 12 (y 0.5-16), F.Cu at x 13.7, In1.Cu and
// In2.Cu at x 11.7 and x 14.1 (y 0.5-19.5). An F.Cu track reaches x <= 12.75, a B.Cu track starts at x >= 12.95, an
// inner track lies in 12.65-13.15 and a via's F.Cu ring stays at x <= 13.1: near y 10 only a hop on two vias 0.2 to
// 0.45 mm apart joins them (holes closer than the 0.2 mm hole-to-hole). Legal routes: a diagonal hop whose vias
// keep 0.5 mm, or the way round the end of the B.Cu wall.
model::Board board() {
  return io::read_board(sexpr::Document::parse(R"((kicad_pcb (version 20240108) (generator "pcbnew")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "") (net 1 "A") (net 2 "W")
  (footprint "T:P" (layer "F.Cu") (at 5 10) (property "Reference" "R1")
    (pad "1" smd rect (at 0 0) (size 1.5 1.5) (layers "F.Cu") (net 1 "A")))
  (footprint "T:P" (layer "B.Cu") (at 25 10) (property "Reference" "R2")
    (pad "1" smd rect (at 0 0) (size 1.5 1.5) (layers "B.Cu") (net 1 "A")))
  (segment (start 12 0.5) (end 12 16) (width 0.2) (locked yes) (layer "B.Cu") (net 2))
  (segment (start 13.7 0.5) (end 13.7 19.5) (width 0.2) (locked yes) (layer "F.Cu") (net 2))
  (segment (start 11.7 0.5) (end 11.7 19.5) (width 0.2) (locked yes) (layer "In1.Cu") (net 2))
  (segment (start 11.7 0.5) (end 11.7 19.5) (width 0.2) (locked yes) (layer "In2.Cu") (net 2))
  (segment (start 14.1 0.5) (end 14.1 19.5) (width 0.2) (locked yes) (layer "In1.Cu") (net 2))
  (segment (start 14.1 0.5) (end 14.1 19.5) (width 0.2) (locked yes) (layer "In2.Cu") (net 2))
  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
))"));
}

model::DesignRules rules() {
  model::DesignRules r;
  r.minimums.clearance = 200'000;
  r.minimums.track_width = 150'000;
  r.minimums.via_diameter = 600'000;
  r.minimums.through_hole_diameter = 300'000;
  r.minimums.hole_clearance = 200'000;
  r.minimums.hole_to_hole = 200'000;
  model::NetClass d;
  d.name = "Default";
  d.clearance = 200'000;
  d.track_width = 1'300'000;
  d.via_diameter = 600'000;
  d.via_drill = 300'000;
  r.classes = {d};
  return r;
}

}  // namespace

TEST_CASE("via spacing: a route's own vias keep hole to hole between each other", "[route][via_spacing]") {
  route::RouterOptions o;
  o.work_budget = 40'000'000;
  o.time_limit_s = 600;
  o.gpu_device = -1;
  o.via_cost_mm = 1.0;  // the farm's: two vias cost less than the legal detour
  const auto r = route::Router(board(), rules(), o).run();
  CHECK(r.connections == 1);
  CHECK(r.routed == 1);  // legal routes exist
  for (std::size_t i = 0; i < r.vias.size(); ++i)
    for (std::size_t j = i + 1; j < r.vias.size(); ++j) {
      const auto& a = r.vias[i];
      const auto& b = r.vias[j];
      const double d = std::hypot(static_cast<double>(a.pos.x - b.pos.x), static_cast<double>(a.pos.y - b.pos.y));
      INFO("vias at (" << a.pos.x << ", " << a.pos.y << ") and (" << b.pos.x << ", " << b.pos.y << ")");
      CHECK(d - static_cast<double>(a.drill + b.drill) / 2 >= 200'000);  // hole to hole
    }
}
