// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Tidy pass (`tracemaker-place --tidy`, doc 04 §11): after the placement is final, make it look like a hand layout
// without making it less legal. Three greedy, deterministic steps, each move of a movable part accepted only where
// the exact legality test (Legality::legal, the part taken out of the index first, as the legaliser does) passes:
//   1. grid snap: the footprint origin to the nearest legal of the four surrounding grid points;
//   2. axis alignment: a part whose body centre is within `align` of a heavier part's (fixed, else larger, else
//      more pins, else lower index) in x, and within 6 mm in y, takes its x exactly (and the same for y); up to 3
//      passes;
//   3. orientation: in clusters of the same two-pad passive footprint (R, C, L, FB, D; bodies < 3 mm apart), the
//      minority turns to the majority axis (0/180 vs 90/270) when the summed HPWL of its nets rises ≤ 0.5 mm.
// Parts keep their side; fixed parts never move.
#include "place/legality.hpp"

namespace tmk::place {

struct TidyOptions {
  Coord grid = 250'000;       // step 1 grid (0 = skip)
  Coord align = 500'000;      // step 2 tolerance (0 = skip)
  bool orient = true;         // step 3
};

struct TidyStats {
  int snapped = 0, aligned = 0, reoriented = 0;  // parts moved by each step
};

TidyStats tidy(const Problem& p, Placement& pl, const TidyOptions& o = {});

}  // namespace tmk::place
