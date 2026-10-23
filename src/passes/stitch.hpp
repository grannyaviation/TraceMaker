// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Stitching vias (doc 05 §39.3): after routing, vias of a plane's net are added where fills of that net lie over
// each other on two or more layers (plane stitching), and next to the vias of signals that need a reference plane
// (return-path stitching). Every via passes the router's exact check against the refilled zones, and is kept only
// if a second refill still joins it to its net's fill on two layers.
#include <string>
#include <vector>

#include "model/board.hpp"
#include "model/rules.hpp"

namespace tmk::passes {

struct StitchOptions {
  bool planes = false;          // plane stitching on a grid
  // Grid pitch. The default is the catalogue's general high-speed value (TI SPRAAR7J: a stitching via within
  // 200 mil); app code passes the catalogue's number, this is its fallback.
  Coord pitch = 5'080'000;
  // Largest empty circle the grid may leave (centre of the board to its nearest via). A site the exact check refuses is
  // moved on rings round it, nearest first, up to stitch_shift_bound(pitch, coverage). 0: the grid's own radius
  // pitch / sqrt 2 plus 0.8 %, which is the farm's gate over its 8.7 mm grid (6.2 mm over 6.15 mm).
  Coord coverage = 0;
  // Return-path stitching: every through via of `signal` wants a through via (or plated through-hole pad) of `plane`
  // within `distance`, centre to centre. Laser and blind vias count on neither side (they tie the layers of their span).
  struct ReturnNet {
    model::NetId signal = 0, plane = 0;
    Coord distance = 5'080'000;
    std::string source;         // "option" or the component rule (e.g. "USB2-09 J1")
  };
  std::vector<ReturnNet> returns;
  // Work budget (rule 5): sites examined plus exact checks. When it runs out no further site is tried.
  long work_limit = 2'000'000;
};

// The coverage radius the pass uses: `opt.coverage`, else the grid's own radius plus 0.8 % (the farm's 6.2 mm gate over its
// 8.7 mm grid's 6.15 mm).
Coord stitch_coverage(const StitchOptions& opt);

struct StitchReturn {
  model::NetId signal = 0, plane = 0;
  std::string source;
  Coord distance = 0;
  int signal_vias = 0;          // through vias of the signal net on the board (old and new); laser and blind vias change no side
  int stitched_before = 0;      // of them: with a plane via or plated pad within the distance before the pass
  int stitched_after = 0;       // ... and after it
};

// A grid site the exact check refused and no point of its ring could take (StitchOptions::coverage bounds the ring).
// The caller lays a stub there or accepts the gap.
struct StitchGap {
  model::NetId net = 0;
  model::Point pos;
};

// How far one site of the square grid of `pitch` may move, in the worst direction (away from a cell diagonally), while
// the largest empty circle on the board stays within `coverage` (the grid itself leaves pitch / sqrt 2). With
// t = d / sqrt 2 the worst radius is (p^2 + 2pt + 2t^2) / (sqrt 2 (p + 2t)), so d = (sqrt 2 R - p + sqrt(2R^2 - p^2)) / sqrt 2.
// 0 when `coverage` is below the grid's own radius. 8.7 mm and 6.2 mm give 0.82 mm.
Coord stitch_shift_bound(Coord pitch, Coord coverage);


struct StitchResult {
  std::vector<model::Via> vias;            // the vias to add, plane stitches first; all marked free
  std::vector<std::uint8_t> return_path;   // per via: 1 = placed for a signal via, 0 = grid
  std::vector<StitchReturn> returns;       // one row per StitchOptions::returns entry
  int sites = 0;                // grid sites inside overlapping fills
  int site_served = 0;          // of them: already tied by a via or plated pad of the net within half the pitch
  int site_blocked = 0;         // of them: refused by the exact check at the site itself
  int site_moved = 0;           // of those: placed on a ring round the site, within the coverage bound
  std::vector<StitchGap> gaps;  // the rest: no ring point was legal; one entry each, in grid order
  int dropped = 0;              // vias placed and then removed because the second refill did not join them on two layers
  long checks = 0;              // the pass's work
  bool out_of_work = false;
  bool discarded = false;       // everything withdrawn: the refill with the vias left another net's zone in more pieces
  std::vector<std::string> warnings;
};

// `board` holds the routed copper (the run's new tracks and vias appended). Zone fills on it may be stale: the
// pass refills them in the engine (drc/refill.hpp) and places against that.
StitchResult stitch_vias(const model::Board& board, const model::DesignRules& rules, const StitchOptions& opt);

}  // namespace tmk::passes
