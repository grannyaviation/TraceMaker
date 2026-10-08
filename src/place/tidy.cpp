// SPDX-License-Identifier: GPL-3.0-or-later
#include "place/tidy.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numeric>

#include "place/wirelength.hpp"

namespace tmk::place {

namespace {

constexpr Coord kAlignReach = 6'000'000;  // step 2: partners within 6 mm on the other axis
constexpr Coord kClusterGap = 3'000'000;  // step 3: bodies closer than this are one cluster
constexpr Coord kOrientSlack = 500'000;   // step 3: summed HPWL of the part's nets may rise this much

Box body_at(const Problem& p, const Placement& pl, int i) {
  const Box& b = p.parts[z(i)].geom[pl.rot[z(i)]].body;
  const Point q = pl.pos[z(i)];
  return Box{b.x0 + q.x, b.y0 + q.y, b.x1 + q.x, b.y1 + q.y};
}

Point centre(const Problem& p, const Placement& pl, int i) {
  const Box b = body_at(p, pl, i);
  return Point{(b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2};
}

Coord floor_to(Coord v, Coord g) {
  const Coord q = v / g;
  return (v % g != 0 && v < 0 ? q - 1 : q) * g;
}

int side_of(const Problem& p, const Placement& pl, int i) { return p.parts[z(i)].side_in(pl.rot[z(i)]); }

// R, C, L, FB or D by reference, with exactly two pads.
bool two_pad_passive(const Part& pt) {
  if (pt.pad_count != 2) return false;
  std::size_t k = 0;
  while (k < pt.ref.size() && std::isalpha(static_cast<unsigned char>(pt.ref[k]))) ++k;
  const std::string pre = pt.ref.substr(0, k);
  return pre == "R" || pre == "C" || pre == "L" || pre == "FB" || pre == "D";
}

// Orientation axis in millidegrees, [0, 180000): 0° and 180° are the same axis.
long axis_of(const Part& pt, int state) { return std::lround(std::fmod(geom::norm_deg(pt.angle_of(state)), 180.0) * 1000) % 180'000; }

// Moves part i to (q, r) if that is exactly legal with i taken out of the index (CLAUDE.md rule 1).
bool try_move(Legality& L, Placement& pl, int i, Point q, int r) {
  L.remove(i);
  const bool ok = L.legal(i, q, r);
  if (ok) {
    pl.pos[z(i)] = q;
    pl.rot[z(i)] = static_cast<std::uint8_t>(r);
  }
  L.insert(i, pl.pos[z(i)], pl.rot[z(i)]);
  return ok;
}

}  // namespace

TidyStats tidy(const Problem& p, Placement& pl, const TidyOptions& o) {
  TidyStats st;
  const int n = static_cast<int>(p.parts.size());
  Legality L(p);
  L.reset(pl);

  // 1. Grid snap: the nearest of the four surrounding grid points that is legal (ties: lower y, then lower x).
  if (o.grid > 0)
    for (int i = 0; i < n; ++i) {
      if (!p.parts[z(i)].movable) continue;
      const Point at = pl.pos[z(i)];
      const Coord x0 = floor_to(at.x, o.grid), y0 = floor_to(at.y, o.grid);
      if (x0 == at.x && y0 == at.y) continue;
      std::vector<Point> cand;
      for (const Coord y : {y0, y0 + o.grid})
        for (const Coord x : {x0, x0 + o.grid})
          if ((x == x0 || x0 != at.x) && (y == y0 || y0 != at.y)) cand.push_back(Point{x, y});
      auto d2 = [&](Point q) { return (q.x - at.x) * (q.x - at.x) + (q.y - at.y) * (q.y - at.y); };
      std::stable_sort(cand.begin(), cand.end(), [&](Point a, Point b) { return d2(a) < d2(b); });
      for (const Point q : cand)
        if (try_move(L, pl, i, q, pl.rot[z(i)])) {
          ++st.snapped;
          break;
        }
    }

  // 2. Axis alignment. A part only ever moves towards a heavier one (a strict total order), so nothing cycles;
  // heaviest movers first, so a light part aligns to where its anchor ended up in the same pass.
  auto heavier = [&](int a, int b) {  // a is the better anchor
    const Part &pa = p.parts[z(a)], &pb = p.parts[z(b)];
    if (pa.movable != pb.movable) return !pa.movable;
    if (pa.area != pb.area) return pa.area > pb.area;
    if (pa.pins.size() != pb.pins.size()) return pa.pins.size() > pb.pins.size();
    return a < b;
  };
  if (o.align > 0) {
    std::vector<int> movers;
    for (int i = 0; i < n; ++i)
      if (p.parts[z(i)].movable) movers.push_back(i);
    std::sort(movers.begin(), movers.end(), heavier);
    std::vector<std::uint8_t> aligned(z(n), 0);
    std::vector<std::pair<int, Coord>> cand;  // anchor, shift
    // ponytail: O(n²) partner scan per pass; use Legality::neighbours if boards reach thousands of parts.
    for (int pass = 0; pass < 3; ++pass) {
      int moves = 0;
      for (const int i : movers)
        for (int ax = 0; ax < 2; ++ax) {
          const Point ci = centre(p, pl, i);
          cand.clear();
          bool done = false;
          for (int j = 0; j < n && !done; ++j) {
            if (j == i || !heavier(j, i) || side_of(p, pl, j) != side_of(p, pl, i)) continue;
            const Point cj = centre(p, pl, j);
            const Coord d = ax == 0 ? cj.x - ci.x : cj.y - ci.y, off = ax == 0 ? cj.y - ci.y : cj.x - ci.x;
            if (std::llabs(off) > kAlignReach || std::llabs(d) >= o.align) continue;
            if (d == 0) done = true;  // already aligned with an anchor: keep it
            else cand.emplace_back(j, d);
          }
          if (done) continue;
          std::sort(cand.begin(), cand.end(), [&](const auto& a, const auto& b) { return heavier(a.first, b.first); });
          for (const auto& [j, d] : cand)
            if (try_move(L, pl, i, pl.pos[z(i)] + (ax == 0 ? Point{d, 0} : Point{0, d}), pl.rot[z(i)])) {
              aligned[z(i)] = 1;
              ++moves;
              break;
            }
        }
      if (moves == 0) break;
    }
    st.aligned = static_cast<int>(std::count(aligned.begin(), aligned.end(), 1));
  }

  // 3. Orientation of two-pad passives: clusters of one footprint on one side, bodies < 3 mm apart (fixed members
  // vote too); the minority turns a quarter to the majority axis if legal and its nets' HPWL rises ≤ 0.5 mm.
  if (o.orient) {
    std::vector<int> passives;
    for (int i = 0; i < n; ++i)
      if (two_pad_passive(p.parts[z(i)])) passives.push_back(i);
    std::vector<int> parent(z(n));
    std::iota(parent.begin(), parent.end(), 0);
    auto root = [&](int a) {
      while (parent[z(a)] != a) {
        parent[z(a)] = parent[z(parent[z(a)])];
        a = parent[z(a)];
      }
      return a;
    };
    for (std::size_t s = 0; s < passives.size(); ++s)
      for (std::size_t t = s + 1; t < passives.size(); ++t) {
        const int a = passives[s], b = passives[t];
        if (p.parts[z(a)].lib_id != p.parts[z(b)].lib_id || side_of(p, pl, a) != side_of(p, pl, b)) continue;
        const Box ba = body_at(p, pl, a), bb = body_at(p, pl, b);
        const Coord gx = std::max<Coord>({0, bb.x0 - ba.x1, ba.x0 - bb.x1}), gy = std::max<Coord>({0, bb.y0 - ba.y1, ba.y0 - bb.y1});
        if (static_cast<geom::i128>(gx) * gx + static_cast<geom::i128>(gy) * gy < static_cast<geom::i128>(kClusterGap) * kClusterGap)
          parent[z(root(a))] = root(b);
      }
    std::vector<std::vector<int>> clusters(z(n));
    for (const int i : passives) clusters[z(root(i))].push_back(i);
    std::vector<int> nets;
    auto hpwl = [&](int i, int r) {
      const std::uint8_t keep = pl.rot[z(i)];
      pl.rot[z(i)] = static_cast<std::uint8_t>(r);
      Coord s = 0;
      for (const int net : nets) s += net_hpwl(p, pl, net);
      pl.rot[z(i)] = keep;
      return s;
    };
    for (const auto& c : clusters) {
      if (c.size() < 3) continue;  // two parts have no majority
      std::vector<std::pair<long, int>> votes;  // axis, count
      for (const int i : c) {
        const long a = axis_of(p.parts[z(i)], pl.rot[z(i)]);
        auto it = std::find_if(votes.begin(), votes.end(), [&](const auto& v) { return v.first == a; });
        if (it == votes.end()) votes.emplace_back(a, 1);
        else ++it->second;
      }
      std::stable_sort(votes.begin(), votes.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
      if (votes.size() < 2 || votes[0].second == votes[1].second) continue;
      const long major = votes[0].first;
      for (const int i : c) {
        const Part& pt = p.parts[z(i)];
        const int rot = pl.rot[z(i)];
        if (!pt.movable || (axis_of(pt, rot) - major + 180'000) % 180'000 != 90'000) continue;
        nets.clear();
        for (const int pi : pt.pins)
          if (!p.nets[z(p.pins[z(pi)].net)].affinity) nets.push_back(p.pins[z(pi)].net);
        std::sort(nets.begin(), nets.end());
        nets.erase(std::unique(nets.begin(), nets.end()), nets.end());
        const Coord before = hpwl(i, rot);
        std::vector<std::pair<Coord, int>> opts;  // HPWL, state
        for (const int dr : {1, 3}) {
          const int r = with_turn(rot, rot + dr);
          if (const Coord h = hpwl(i, r); h - before <= kOrientSlack) opts.emplace_back(h, r);
        }
        std::stable_sort(opts.begin(), opts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        for (const auto& opt : opts)
          if (try_move(L, pl, i, pl.pos[z(i)], opt.second)) {
            ++st.reoriented;
            break;
          }
      }
    }
  }
  return st;
}

}  // namespace tmk::place
