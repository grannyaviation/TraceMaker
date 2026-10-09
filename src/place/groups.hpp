// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Groups (`tracemaker-place --groups FILE`, layout-farm clean-layout spec S2): parts that move as one rigid unit, e.g.
// an IC with the passives the farm arranged round it. Each group becomes one composite part: the leader (the first
// reference) takes every member's courtyards, holes, copper and pins, offset by the member's input position relative
// to the leader's and turned with it; a member keeps no geometry and no pins and follows its leader.
#include <string>
#include <vector>

#include "place/legality.hpp"

namespace tmk::place {

// Merges each group (leader first) whose parts all exist, are movable, are on the leader's side, are listed once and
// belong to no other group. Other groups are left as they are. Returns how many were merged.
int merge_groups(Problem& p, const std::vector<std::vector<std::string>>& groups);

// Sets every group member's pose from its leader's: origin = leader origin + rot90(member offset, leader turns),
// state = the leader's quarter turns. Parts outside groups are untouched.
void place_followers(const Problem& p, Placement& pl);

// Reads a --groups file: a JSON array of arrays of references, leader first. Throws when it cannot be read.
std::vector<std::vector<std::string>> read_groups(const std::string& path);

}  // namespace tmk::place
