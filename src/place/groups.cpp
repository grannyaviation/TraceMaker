// SPDX-License-Identifier: GPL-3.0-or-later
#include "place/groups.hpp"

#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace tmk::place {

namespace {

Box shifted(const Box& b, Point d) { return b.empty() ? b : Box{b.x0 + d.x, b.y0 + d.y, b.x1 + d.x, b.y1 + d.y}; }

// Adds the member geometry g (offsets from the member's origin), moved by d, to the leader's geometry lg.
void absorb(PartGeom& lg, const PartGeom& g, Point d) {
  for (int s = 0; s < 2; ++s) {
    for (const auto& sh : g.cy[z(s)]) lg.cy[z(s)].push_back(translated(sh, d));
    for (const auto& sh : g.cy_in[z(s)]) lg.cy_in[z(s)].push_back(translated(sh, d));
  }
  for (const auto& sh : g.through) lg.through.push_back(translated(sh, d));
  for (const auto& sh : g.pads) lg.pads.push_back(translated(sh, d));
  for (const auto& cs : g.copper) lg.copper.push_back(CopperShape{translated(cs.s, d), cs.layers, cs.net, cs.need});
  lg.copper_box.add(shifted(g.copper_box, d));
  lg.body.add(shifted(g.body, d));
  lg.edge_box.add(shifted(g.edge_box, d));
}

}  // namespace

int merge_groups(Problem& p, const std::vector<std::vector<std::string>>& groups) {
  std::map<std::string, int> index;
  for (std::size_t i = 0; i < p.parts.size(); ++i) index[p.parts[i].ref] = static_cast<int>(i);
  int merged = 0;
  for (const auto& g : groups) {
    if (g.size() < 2) continue;
    std::vector<int> ix;
    std::set<int> seen;
    bool ok = true;
    for (const auto& r : g) {
      const auto it = index.find(r);
      if (it == index.end() || !seen.insert(it->second).second) {
        ok = false;
        break;
      }
      ix.push_back(it->second);
    }
    if (!ok) continue;
    const int lead_i = ix.front();
    for (int i : ix) {
      const Part& pt = p.parts[z(i)];
      if (!pt.movable || pt.leader >= 0 || pt.side != p.parts[z(lead_i)].side) ok = false;
    }
    if (!ok) continue;
    Part& lead = p.parts[z(lead_i)];
    for (std::size_t k = 1; k < ix.size(); ++k) {
      Part& m = p.parts[z(ix[k])];
      const Point d = m.pos0 - lead.pos0;
      for (int r = 0; r < 4; ++r) absorb(lead.geom[z(r)], m.geom[z(r)], rot90(d, r));
      for (int pin : m.pins) {
        Pin& q = p.pins[z(pin)];
        q.part = lead_i;
        for (int r = 0; r < 4; ++r) q.off[z(r)] = rot90(d, r) + q.off[z(r)];
        lead.pins.push_back(pin);
      }
      m.pins.clear();
      for (auto& mg : m.geom) mg = PartGeom{};
      m.movable = false;
      m.flippable = false;
      m.fixed_reason = "group member of " + lead.ref;
      m.leader = lead_i;
      m.group_off = d;
      lead.area = m.area > std::numeric_limits<Coord>::max() - lead.area ? std::numeric_limits<Coord>::max() : lead.area + m.area;
      m.area = 0;
    }
    // A composite keeps its side (no flipped states) and is interchangeable with nothing (no swap moves).
    lead.flippable = false;
    for (int s = 4; s < kStates; ++s) lead.geom[z(s)] = PartGeom{};
    lead.shape_key = std::hash<std::string>{}("group:" + lead.ref);
    ++merged;
  }
  return merged;
}

void place_followers(const Problem& p, Placement& pl) {
  for (std::size_t i = 0; i < p.parts.size(); ++i) {
    const Part& pt = p.parts[i];
    if (pt.leader < 0) continue;
    const auto lead = z(pt.leader);
    const int r = pl.rot[lead] & 3;
    pl.pos[i] = pl.pos[lead] + rot90(pt.group_off, r);
    pl.rot[i] = static_cast<std::uint8_t>(r);
  }
}

std::vector<std::vector<std::string>> read_groups(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot read --groups file " + path);
  const auto j = nlohmann::json::parse(f);
  std::vector<std::vector<std::string>> out;
  for (const auto& g : j) out.push_back(g.get<std::vector<std::string>>());
  return out;
}

}  // namespace tmk::place
