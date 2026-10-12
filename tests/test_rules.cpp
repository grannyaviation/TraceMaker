// SPDX-License-Identifier: GPL-3.0-or-later
#include <catch2/catch_test_macros.hpp>

#include <filesystem>

#include "drc/copper.hpp"
#include "drc/rule_engine.hpp"
#include "io/kicad/board_reader.hpp"
#include "io/kicad/project_reader.hpp"

namespace fs = std::filesystem;
using tmk::io::parse_length;
using tmk::io::read_design_rules;

TEST_CASE("lengths with units convert to nanometres", "[rules]") {
  CHECK(parse_length("0.2mm") == 200'000);
  CHECK(parse_length("0.2") == 200'000);
  CHECK(parse_length("8mil") == 203'200);
  CHECK(parse_length("0.01in") == 254'000);
  CHECK(parse_length("150um") == 150'000);
  CHECK_FALSE(parse_length("abc").has_value());
}

TEST_CASE("net-class wildcard patterns", "[rules]") {
  using tmk::model::wildcard_match;
  CHECK(wildcard_match("+12V", "+12V"));
  CHECK(wildcard_match("*USB*", "/usb/USB_D+"));
  CHECK(wildcard_match("85Ohm-diff*", "85Ohm-diff_DP"));
  CHECK(wildcard_match("D?", "D1"));
  CHECK_FALSE(wildcard_match("D?", "D10"));
  CHECK_FALSE(wildcard_match("*USB*", "/usb/D+"));
}

TEST_CASE("project net classes, patterns and minimums are read", "[rules]") {
  const fs::path b = fs::path(TM_SOURCE_DIR) / "bench/data/kicad/demos/video/video.kicad_pcb";
  if (!fs::exists(b)) SKIP("fixtures missing");
  const auto r = read_design_rules(b.string());
  CHECK(r.warnings.empty());
  REQUIRE(r.classes.size() == 2);
  CHECK(r.default_class().name == "Default");
  CHECK(r.default_class().clearance == 200'000);
  CHECK(r.default_class().via_diameter == 889'000);
  CHECK(r.class_for("+12V").name == "pwr");
  CHECK(r.class_for("+12V").track_width == 250'000);
  CHECK(r.class_for("SOMETHING").name == "Default");
  CHECK(r.minimums.track_width == 200'000);
  CHECK(r.minimums.hole_to_hole == 250'000);
  CHECK(r.minimums.copper_edge_clearance == 10'000);
}

TEST_CASE("classes inherit null values from Default; explicit assignments and custom rules are read", "[rules]") {
  const fs::path b = fs::path(TM_SOURCE_DIR) / "bench/data/kicad/demos/jetson-agx-thor-baseboard/jetson-agx-thor-baseboard.kicad_pcb";
  if (!fs::exists(b)) SKIP("fixtures missing");
  const auto r = read_design_rules(b.string());
  CHECK(r.warnings.empty());
  const auto* dp = r.find_class("85Ohm-diff_DP");
  REQUIRE(dp != nullptr);
  CHECK(dp->clearance == r.default_class().clearance);
  CHECK(dp->priority == 1);
  CHECK(r.class_for("/CSI/CAM0.CSI0_CLK+").name == "90Ohm-diff_CSI");
  REQUIRE(r.custom.size() == 5);
  CHECK(r.custom[0].layer == "inner");
  CHECK(r.custom[0].condition == "(A.NetClass == '85Ohm-diff*')");
  REQUIRE(r.custom[0].constraints.size() == 2);
  CHECK(r.custom[0].constraints[0].type == "track_width");
  CHECK(r.custom[0].constraints[0].opt == 150'000);
}

TEST_CASE("custom rule conditions compare strings case-insensitively with wildcards on either side", "[rules]") {
  using namespace tmk;
  const auto b = io::read_board(sexpr::Document::parse(
      "(kicad_pcb (version 20260624) (layers (0 \"F.Cu\" signal) (4 \"In1.Cu\" signal) (6 \"In2.Cu\" signal) (2 \"B.Cu\" signal)) "
      "(net 0 \"\") (net 1 \"CLK\") (net 2 \"DATA*\") (net 3 \"usb_dp\"))"));
  auto rules = read_design_rules("/nonexistent/board.kicad_pcb");
  model::NetClass hs = rules.default_class();
  hs.name = "HighSpeed";
  rules.classes.push_back(hs);
  rules.assignments["usb_dp"] = {"HighSpeed"};
  const auto cm = drc::build_copper(b);
  // Does a 0.2 mm track of `net` on copper layer `layer` match the rule (condition and layer selector)?
  auto match = [&](const char* cond, model::NetId net, int layer = -1, const char* sel = "") {
    rules.custom = {model::CustomRule{"r", cond, sel, "", {model::Constraint{"track_width", 123, {}, {}, {}}}}};
    const drc::RuleEngine eng(b, rules, cm);
    drc::CopperItem t;
    t.kind = drc::ItemKind::Track;
    t.net = net;
    t.width = 200'000;
    return eng.track_width(t, layer).first == 123;
  };
  CHECK(match("A.NetName == 'clk'", 1));  // case-insensitive
  CHECK(match("'Clk' == A.NetName", 1));
  CHECK_FALSE(match("A.NetName == 'CL'", 1));  // a prefix is no match, nor is a longer name
  CHECK_FALSE(match("A.NetName == 'CLKX'", 1));
  CHECK_FALSE(match("A.NetName != 'clk'", 1));
  CHECK(match("A.NetName != 'clk'", 3));
  CHECK(match("A.NetName == 'c*'", 1));  // wildcards in the literal
  CHECK(match("A.NetName == 'C?k'", 1));
  CHECK_FALSE(match("A.NetName == 'D*'", 1));
  CHECK(match("A.NetName == 'data_7'", 2));  // and in the property: net DATA*
  CHECK(match("A.NetName == 'Data'", 2));
  CHECK_FALSE(match("A.NetName == 'clk'", 2));
  CHECK(match("A.NetClass == 'highspeed'", 3));  // a mixed-case class
  CHECK(match("A.NetClass == 'HIGH*'", 3));
  CHECK_FALSE(match("A.NetClass == 'High'", 3));
  CHECK_FALSE(match("A.NetClass == 'HighSpeed'", 1));
  CHECK(match("A.NetClass == 'DEFAULT'", 1));
  CHECK(match("A.NetName == ''", 0));  // empty strings
  CHECK_FALSE(match("A.NetName == ''", 1));
  CHECK(match("A.NetName != ''", 1));
  CHECK(match("A.NetName == '*'", 0));
  CHECK(match("A.Type == 'TRACK' && A.Type != 'Via'", 1));
  CHECK(match("A.Width == '0.2'", 1));
  CHECK(match("A.Layer == 'in1.cu'", 1, 1));
  CHECK(match("A.Layer == ''", 1));
  // Layer selectors: outer, inner, a layer name; every selector applies without a layer.
  CHECK(match("", 1, 0, "outer"));
  CHECK(match("", 1, 3, "outer"));
  CHECK_FALSE(match("", 1, 1, "outer"));
  CHECK(match("", 1, 2, "inner"));
  CHECK_FALSE(match("", 1, 3, "inner"));
  CHECK(match("", 1, 2, "In2.Cu"));
  CHECK_FALSE(match("", 1, 1, "In2.Cu"));
  CHECK(match("", 1, -1, "In2.Cu"));
  // A rule on one item's class applies to a pair either way round, and not to other classes.
  rules.custom = {model::CustomRule{"hs", "A.NetClass == 'HIGHSPEED' || A.NetClass == 'Pwr*'", "", "", {model::Constraint{"clearance", 777, {}, {}, {}}}}};
  const drc::RuleEngine eng(b, rules, cm);
  drc::CopperItem clk, usb;
  clk.kind = usb.kind = drc::ItemKind::Track;
  clk.net = 1;
  usb.net = 3;
  CHECK(eng.clearance(clk, usb, 0) == 777);
  CHECK(eng.clearance(usb, clk, 0) == 777);
  CHECK(eng.clearance(clk, clk, 0) == rules.default_class().clearance);
}
