// SPDX-License-Identifier: GPL-3.0-or-later
#include "drc/rule_engine.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace tmk::drc {

// ---------------------------------------------------------------------------------------------------------
// Condition expressions: a small recursive-descent parser for the subset of KiCad's rule language used in
// practice: A./B. properties, string/number literals, == != && || ! and parentheses, and a few functions.
// ---------------------------------------------------------------------------------------------------------
struct EvalCtx {
  const RuleEngine* eng;
  const CopperItem* a;
  const CopperItem* b;
  int layer;
  bool unknown = false;  // set when an unsupported property or function is evaluated
};

// A property or function a condition reads, resolved when the condition is parsed (the router evaluates
// conditions for every clearance query: comparing names there took most of its time).
enum class Fn : std::uint8_t {
  Unknown,
  NetClass, NetName, Type, Layer, Reference, PadType, Width,  // properties
  IsPlated, ExistsOnLayer, InsideArea, IntersectsArea, EnclosedByArea, InDiffPair, Courtyard, FrontCourtyard,
  BackCourtyard, MemberOfFootprint,  // functions
};

struct Value {
  enum class K { Undef, Bool, Str, Num } k = K::Undef;
  bool b = false;
  std::string_view s;  // into storage that outlives the evaluation: a literal, a net, class, footprint, kind or layer name
  double n = 0;
  signed char wild = -1;  // s has a '*' or '?': 1 or 0 when known, -1 to look
  bool truthy() const { return k == K::Bool ? b : k == K::Num ? n != 0 : k == K::Str ? !s.empty() : false; }
};

static bool has_wildcard(const Value& v) {
  if (v.wild >= 0) return v.wild != 0;
  return std::any_of(v.s.begin(), v.s.end(), [](char c) { return c == '*' || c == '?'; });
}

static bool iequal(std::string_view a, std::string_view b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (model::ascii_lower(a[i]) != model::ascii_lower(b[i])) return false;
  return true;
}

class Condition {
 public:
  struct Node {
    enum class Op { Or, And, Not, Eq, Ne, Lit, Prop, Call } op;
    std::vector<std::unique_ptr<Node>> kids;
    Value lit;
    std::string text;     // a string literal (lit.s views it)
    char who = 'A';       // A or B
    std::string name;     // property or function name
    Fn fn = Fn::Unknown;
    std::vector<std::string> args;
    int arg = -1;  // existsOnLayer: copper layer; courtyard functions: footprint index, -1 none, -2 a wildcard pattern
  };

  // `eng` resolves layer and footprint arguments; its board and footprint index must be complete.
  static std::unique_ptr<Condition> parse(const std::string& text, const RuleEngine& eng, std::string& err) {
    auto c = std::make_unique<Condition>();
    c->eng_ = &eng;
    c->src_ = text;
    c->pos_ = 0;
    try {
      c->root_ = c->parse_or();
      c->skip();
      if (c->pos_ != c->src_.size()) throw std::runtime_error("unexpected '" + c->src_.substr(c->pos_, 10) + "'");
    } catch (const std::exception& e) {
      err = e.what();
      return nullptr;
    }
    return c;
  }

  bool eval(EvalCtx& ctx) const { return eval(*root_, ctx).truthy(); }
  // False when no item of net class `cls` can satisfy the condition as A: A.NetClass compared with a literal is
  // known, everything else unknown (Kleene logic). custom_min then skips most class rules without evaluating them.
  bool may_hold(std::string_view cls) const { return tri(*root_, cls) != 0; }

 private:
  void skip() {
    while (pos_ < src_.size() && std::isspace(static_cast<unsigned char>(src_[pos_]))) ++pos_;
  }
  bool eat(std::string_view tok) {
    skip();
    if (src_.compare(pos_, tok.size(), tok) == 0) {
      pos_ += tok.size();
      return true;
    }
    return false;
  }
  std::unique_ptr<Node> make(Node::Op op) {
    auto n = std::make_unique<Node>();
    n->op = op;
    return n;
  }
  std::unique_ptr<Node> parse_or() {
    auto l = parse_and();
    while (eat("||")) {
      auto n = make(Node::Op::Or);
      n->kids.push_back(std::move(l));
      n->kids.push_back(parse_and());
      l = std::move(n);
    }
    return l;
  }
  std::unique_ptr<Node> parse_and() {
    auto l = parse_unary();
    while (eat("&&")) {
      auto n = make(Node::Op::And);
      n->kids.push_back(std::move(l));
      n->kids.push_back(parse_unary());
      l = std::move(n);
    }
    return l;
  }
  std::unique_ptr<Node> parse_unary() {
    skip();
    if (pos_ < src_.size() && src_[pos_] == '!' && (pos_ + 1 >= src_.size() || src_[pos_ + 1] != '=')) {
      ++pos_;
      auto n = make(Node::Op::Not);
      n->kids.push_back(parse_unary());
      return n;
    }
    auto l = parse_term();
    if (eat("==")) {
      auto n = make(Node::Op::Eq);
      n->kids.push_back(std::move(l));
      n->kids.push_back(parse_term());
      return n;
    }
    if (eat("!=")) {
      auto n = make(Node::Op::Ne);
      n->kids.push_back(std::move(l));
      n->kids.push_back(parse_term());
      return n;
    }
    return l;
  }
  std::string read_string() {
    const char q = src_[pos_++];
    std::string s;
    while (pos_ < src_.size() && src_[pos_] != q) s += src_[pos_++];
    if (pos_ >= src_.size()) throw std::runtime_error("unterminated string");
    ++pos_;
    return s;
  }
  std::unique_ptr<Node> parse_term() {
    skip();
    if (pos_ >= src_.size()) throw std::runtime_error("unexpected end");
    const char c = src_[pos_];
    if (c == '(') {
      ++pos_;
      auto n = parse_or();
      if (!eat(")")) throw std::runtime_error("missing ')'");
      return n;
    }
    if (c == '\'' || c == '"') {
      auto n = make(Node::Op::Lit);
      n->text = read_string();
      n->lit.k = Value::K::Str;
      n->lit.s = n->text;
      n->lit.wild = n->text.find_first_of("*?") != std::string::npos;
      return n;
    }
    if (std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '.') {
      std::size_t b = pos_;
      while (pos_ < src_.size() && (std::isalnum(static_cast<unsigned char>(src_[pos_])) || src_[pos_] == '.' || src_[pos_] == '-')) ++pos_;
      auto n = make(Node::Op::Lit);
      n->lit.k = Value::K::Num;
      n->lit.n = std::atof(src_.substr(b, pos_ - b).c_str());
      return n;
    }
    // Identifier: A.Prop, B.func(args), or a bare word (true/false).
    std::size_t b = pos_;
    while (pos_ < src_.size() && (std::isalnum(static_cast<unsigned char>(src_[pos_])) || src_[pos_] == '_' || src_[pos_] == '.')) ++pos_;
    std::string id = src_.substr(b, pos_ - b);
    if (id.empty()) throw std::runtime_error(std::string("unexpected '") + c + "'");
    if (id == "true" || id == "false") {
      auto n = make(Node::Op::Lit);
      n->lit.k = Value::K::Bool;
      n->lit.b = id == "true";
      return n;
    }
    auto n = make(Node::Op::Prop);
    if (id.size() > 2 && (id[0] == 'A' || id[0] == 'B') && id[1] == '.') {
      n->who = id[0];
      n->name = id.substr(2);
    } else {
      n->name = id;
    }
    if (eat("(")) {
      n->op = Node::Op::Call;
      skip();
      while (!eat(")")) {
        skip();
        if (pos_ < src_.size() && (src_[pos_] == '\'' || src_[pos_] == '"')) n->args.push_back(read_string());
        else throw std::runtime_error("function arguments must be strings");
        eat(",");
      }
    }
    resolve(*n);
    return n;
  }

  void resolve(Node& n) const {
    const std::string& s = n.name;
    if (n.op == Node::Op::Prop) {
      n.fn = s == "NetClass"                              ? Fn::NetClass
             : s == "NetName"                             ? Fn::NetName
             : s == "Type"                                ? Fn::Type
             : s == "Layer"                               ? Fn::Layer
             : s == "Reference" || s == "Parent.Reference" ? Fn::Reference
             : s == "Pad_Type"                            ? Fn::PadType
             : s == "Width"                               ? Fn::Width
                                                          : Fn::Unknown;
      return;
    }
    if (s == "isPlated") {
      n.fn = Fn::IsPlated;
      return;
    }
    if (n.args.empty()) return;  // the other functions need their argument
    n.fn = s == "existsOnLayer"              ? Fn::ExistsOnLayer
           : s == "insideArea"               ? Fn::InsideArea
           : s == "intersectsArea"           ? Fn::IntersectsArea
           : s == "enclosedByArea"           ? Fn::EnclosedByArea
           : s == "inDiffPair"               ? Fn::InDiffPair
           : s == "insideCourtyard"          ? Fn::Courtyard
           : s == "intersectsCourtyard"      ? Fn::Courtyard
           : s == "intersectsFrontCourtyard" ? Fn::FrontCourtyard
           : s == "intersectsBackCourtyard"  ? Fn::BackCourtyard
           : s == "memberOfFootprint"        ? Fn::MemberOfFootprint
                                             : Fn::Unknown;
    const std::string& a = n.args[0];
    if (n.fn == Fn::ExistsOnLayer) n.arg = eng_->b_.copper_index(a);
    if (n.fn == Fn::Courtyard || n.fn == Fn::FrontCourtyard || n.fn == Fn::BackCourtyard) {
      if (a.find_first_of("*?") != std::string::npos) n.arg = -2;
      else if (const auto f = eng_->fp_by_ref_.find(a); f != eng_->fp_by_ref_.end()) n.arg = f->second;
    }
  }

  static bool str_eq(const Value& l, const Value& r) {
    if (l.k == Value::K::Num || r.k == Value::K::Num) {
      // atof needs a terminated copy; a number meets a string rarely (A.Width == '0.2').
      const double a = l.k == Value::K::Num ? l.n : std::atof(std::string(l.s).c_str());
      const double b = r.k == Value::K::Num ? r.n : std::atof(std::string(r.s).c_str());
      return std::fabs(a - b) < 1e-9;
    }
    if (l.k == Value::K::Bool || r.k == Value::K::Bool) return l.truthy() == r.truthy();
    // KiCad compares strings case-insensitively with wildcard support on either side. Equal strings match each
    // other as patterns, and without a wildcard a match is equality: most comparisons end at the length check.
    if (iequal(l.s, r.s)) return true;
    if (!has_wildcard(l) && !has_wildcard(r)) return false;
    return model::wildcard_match_nocase(r.s, l.s) || model::wildcard_match_nocase(l.s, r.s);
  }

  // 0 false, 1 true, 2 unknown.
  static int tri(const Node& n, std::string_view cls) {
    switch (n.op) {
      case Node::Op::Or: {
        const int l = tri(*n.kids[0], cls), r = tri(*n.kids[1], cls);
        return l == 1 || r == 1 ? 1 : l == 0 && r == 0 ? 0 : 2;
      }
      case Node::Op::And: {
        const int l = tri(*n.kids[0], cls), r = tri(*n.kids[1], cls);
        return l == 0 || r == 0 ? 0 : l == 1 && r == 1 ? 1 : 2;
      }
      case Node::Op::Not: {
        const int v = tri(*n.kids[0], cls);
        return v == 2 ? 2 : 1 - v;
      }
      case Node::Op::Eq:
      case Node::Op::Ne: {
        auto is_class = [](const Node& k) { return k.op == Node::Op::Prop && k.fn == Fn::NetClass && k.who == 'A'; };
        const Node &l = *n.kids[0], &r = *n.kids[1];
        Value c;
        c.k = Value::K::Str;
        c.s = cls;
        bool eq;
        if (is_class(l) && r.op == Node::Op::Lit) eq = str_eq(c, r.lit);
        else if (l.op == Node::Op::Lit && is_class(r)) eq = str_eq(l.lit, c);
        else return 2;
        return eq == (n.op == Node::Op::Eq) ? 1 : 0;
      }
      case Node::Op::Lit: return n.lit.truthy() ? 1 : 0;
      default: return 2;
    }
  }

  Value prop(const Node& n, EvalCtx& ctx) const {
    const CopperItem* it = n.who == 'A' ? ctx.a : ctx.b;
    Value v;
    if (!it) return v;  // B absent (single-item constraint): undefined → false
    const auto& b = ctx.eng->b_;
    v.k = Value::K::Str;
    switch (n.fn) {
      case Fn::NetClass: v.s = ctx.eng->netclass(*it).name; break;
      case Fn::NetName: v.s = b.nets[static_cast<std::size_t>(it->net)].name; break;
      case Fn::Type:
        v.s = it->kind == ItemKind::Arc ? "Track" : kind_name(it->kind);
        v.wild = 0;
        break;
      case Fn::Layer: v.s = ctx.layer >= 0 ? std::string_view(b.copper_name(ctx.layer)) : std::string_view(); break;
      case Fn::Reference:
        if (it->footprint >= 0) v.s = b.footprints[static_cast<std::size_t>(it->footprint)].reference;
        break;
      case Fn::PadType:
        if (it->kind == ItemKind::Pad) {
          static const char* names[] = {"SMD", "Through-hole", "NPTH, mechanical", "Edge connector"};
          v.s = names[static_cast<int>(b.pads[static_cast<std::size_t>(it->index)].type)];
          v.wild = 0;
          break;
        }
        ctx.unknown = true;
        return Value{};
      case Fn::Width:
        v.k = Value::K::Num;
        v.n = static_cast<double>(it->width) / 1e6;
        break;
      default:
        ctx.unknown = true;
        return Value{};
    }
    return v;
  }

  Value call(const Node& n, EvalCtx& ctx) const {
    const CopperItem* it = n.who == 'A' ? ctx.a : ctx.b;
    Value v;
    v.k = Value::K::Bool;
    if (!it) return v;
    const auto& b = ctx.eng->b_;
    if (n.fn == Fn::IsPlated) {
      v.b = it->kind == ItemKind::Via || (it->kind == ItemKind::Pad && b.pads[static_cast<std::size_t>(it->index)].type == model::PadType::ThruHole);
    } else if (n.fn == Fn::ExistsOnLayer) {
      v.b = n.arg >= 0 && (it->layers & model::layer_bit(n.arg));
    } else if (n.fn == Fn::InsideArea || n.fn == Fn::IntersectsArea || n.fn == Fn::EnclosedByArea) {
      const bool fill = it->kind == ItemKind::Zone;
      const std::tuple<const void*, int, int> key{&n, it->index, it->sub};
      if (fill) {
        const std::lock_guard lock(ctx.eng->area_mutex_);
        if (const auto f = ctx.eng->area_cache_.find(key); f != ctx.eng->area_cache_.end()) {
          v.b = f->second;
          return v;
        }
      }
      for (const auto& z : b.zones) {
        if (!model::wildcard_match(n.args[0], z.name) || z.outline.empty()) continue;
        const auto& poly = z.outline.front();
        const bool all_in = geom::point_in_polygon({it->box.x0, it->box.y0}, poly) && geom::point_in_polygon({it->box.x1, it->box.y1}, poly) &&
                            geom::point_in_polygon({it->box.x0, it->box.y1}, poly) && geom::point_in_polygon({it->box.x1, it->box.y0}, poly);
        bool any = geom::point_in_polygon(it->pos, poly);
        if (!any) {
          const geom::Shape area = geom::Shape::polygon(poly, 0);
          for (const auto& s : it->shapes)
            if (geom::closer_than(s, area, 1)) { any = true; break; }
        }
        if (n.fn == Fn::IntersectsArea ? any : all_in) { v.b = true; break; }
      }
      if (fill) {
        const std::lock_guard lock(ctx.eng->area_mutex_);
        ctx.eng->area_cache_[key] = v.b;
      }
    } else if (n.fn == Fn::InDiffPair) {
      // KiCad: true when the item's net is one half of a differential pair whose base name (without the final
      // P/N or +/-) matches the pattern.
      const auto net = static_cast<std::size_t>(it->net);
      if (it->net != 0 && net < ctx.eng->dp_partner_.size() && ctx.eng->dp_partner_[net] != 0) {
        const std::string_view name = b.nets[net].name;
        v.b = model::wildcard_match(n.args[0], name.substr(0, name.size() - 1)) || model::wildcard_match(n.args[0], name);
      }
    } else if (n.fn == Fn::Courtyard || n.fn == Fn::FrontCourtyard || n.fn == Fn::BackCourtyard) {
      // KiCad: true when any part of the item lies within the named footprint's courtyard, the one on its own
      // side unless Front/Back says which (insideCourtyard is the older name of intersectsCourtyard).
      auto test = [&](std::size_t fi) {
        const auto& fp = b.footprints[fi];
        const int side = n.fn == Fn::FrontCourtyard ? 0 : n.fn == Fn::BackCourtyard ? 1 : fp.back ? 1 : 0;
        for (const auto& cy : ctx.eng->courtyard_[fi][static_cast<std::size_t>(side)]) {
          if (!cy.box.inflated(1).intersects(it->box)) continue;
          if (geom::point_in_polygon(it->pos, cy.pts)) return true;
          // Overlap by 1 um, not a touch: KiCad does not count copper that only meets the outline (a track laid
          // tangent to a courtyard got its exception from the router and the class clearance from KiCad).
          for (const auto& s : it->shapes)
            if (geom::closer_than(s, cy, -1'000)) return true;
        }
        return false;
      };
      if (n.arg != -2) {
        v.b = n.arg >= 0 && test(static_cast<std::size_t>(n.arg));
      } else {
        for (std::size_t fi = 0; fi < b.footprints.size() && !v.b; ++fi)
          v.b = model::wildcard_match(n.args[0], b.footprints[fi].reference) && test(fi);
      }
    } else if (n.fn == Fn::MemberOfFootprint) {
      v.b = it->footprint >= 0 && model::wildcard_match(n.args[0], b.footprints[static_cast<std::size_t>(it->footprint)].reference);
    } else {
      ctx.unknown = true;
      v.k = Value::K::Undef;
    }
    return v;
  }

  Value eval(const Node& n, EvalCtx& ctx) const {
    Value v;
    v.k = Value::K::Bool;
    switch (n.op) {
      case Node::Op::Or: v.b = eval(*n.kids[0], ctx).truthy() || eval(*n.kids[1], ctx).truthy(); return v;
      case Node::Op::And: v.b = eval(*n.kids[0], ctx).truthy() && eval(*n.kids[1], ctx).truthy(); return v;
      case Node::Op::Not: v.b = !eval(*n.kids[0], ctx).truthy(); return v;
      case Node::Op::Eq: v.b = str_eq(eval(*n.kids[0], ctx), eval(*n.kids[1], ctx)); return v;
      case Node::Op::Ne: v.b = !str_eq(eval(*n.kids[0], ctx), eval(*n.kids[1], ctx)); return v;
      case Node::Op::Lit: return n.lit;
      case Node::Op::Prop: return prop(n, ctx);
      case Node::Op::Call: return call(n, ctx);
    }
    return v;
  }

  const RuleEngine* eng_ = nullptr;  // while parsing
  std::string src_;
  std::size_t pos_ = 0;
  std::unique_ptr<Node> root_;
};

// ---------------------------------------------------------------------------------------------------------

namespace {
// Chains open courtyard pieces (lines, arcs as polylines) end to end into closed outlines, ends within 1 um.
// False when some piece does not close up; the caller then falls back to their convex hull.
bool chain_loops(const std::vector<std::vector<geom::Point>>& pieces, std::vector<std::vector<geom::Point>>& loops) {
  auto near = [](geom::Point p, geom::Point q) { return std::abs(p.x - q.x) <= 1000 && std::abs(p.y - q.y) <= 1000; };
  std::vector<char> used(pieces.size(), 0);
  for (std::size_t s = 0; s < pieces.size(); ++s) {
    if (used[s]) continue;
    used[s] = 1;
    std::vector<geom::Point> loop = pieces[s];
    while (loop.size() < 3 || !near(loop.front(), loop.back())) {
      bool found = false;
      for (std::size_t k = 0; k < pieces.size() && !found; ++k) {
        if (used[k]) continue;
        const auto& q = pieces[k];
        if (near(q.front(), loop.back())) loop.insert(loop.end(), q.begin() + 1, q.end());
        else if (near(q.back(), loop.back())) loop.insert(loop.end(), q.rbegin() + 1, q.rend());
        else continue;
        used[k] = 1;
        found = true;
      }
      if (!found) return false;
    }
    loop.pop_back();  // the closing point repeats the first
    loops.push_back(std::move(loop));
  }
  return true;
}
}  // namespace

RuleEngine::RuleEngine(const model::Board& b, const model::DesignRules& r, const CopperModel& cm) : b_(b), r_(r), cm_(cm) {
  // Courtyards for the courtyard functions: closed graphics as they are, lines and arcs chained into closed outlines
  // (a stepped courtyard is not convex: its hull would take in copper KiCad sees outside), their hull only when they
  // do not close up.
  courtyard_.resize(b_.footprints.size());
  {
    std::vector<std::array<std::vector<std::vector<geom::Point>>, 2>> open(b_.footprints.size());
    std::vector<std::array<std::vector<std::vector<geom::Point>>, 2>> closed(b_.footprints.size());
    for (const auto& g : b_.graphics) {
      const int side = g.layer == "F.CrtYd" ? 0 : g.layer == "B.CrtYd" ? 1 : -1;
      if (side < 0 || g.footprint < 0) continue;
      const auto fi = static_cast<std::size_t>(g.footprint);
      const auto si = static_cast<std::size_t>(side);
      switch (g.kind) {
        case model::Graphic::Kind::Line: open[fi][si].push_back({g.a, g.b}); break;
        case model::Graphic::Kind::Arc: open[fi][si].push_back(geom::arc_points(g.a, g.c, g.b)); break;
        case model::Graphic::Kind::Circle:
          closed[fi][si].push_back(geom::circle_points(g.a, geom::kiround(std::hypot(static_cast<double>(g.b.x - g.a.x), static_cast<double>(g.b.y - g.a.y)))));
          break;
        default: closed[fi][si].push_back(g.pts); break;
      }
    }
    for (std::size_t fi = 0; fi < b_.footprints.size(); ++fi)
      for (std::size_t si = 0; si < 2; ++si) {
        auto& pieces = open[fi][si];
        if (!pieces.empty() && !chain_loops(pieces, closed[fi][si])) {
          std::vector<geom::Point> all;
          for (const auto& q : pieces) all.insert(all.end(), q.begin(), q.end());
          auto hull = geom::convex_hull(std::move(all));
          if (hull.size() >= 3) courtyard_[fi][si].push_back(geom::Shape::polygon(std::move(hull), 0));
        }
        for (auto& pts : closed[fi][si])
          if (pts.size() >= 3) courtyard_[fi][si].push_back(geom::Shape::polygon(std::move(pts), 0));
      }
    for (std::size_t fi = 0; fi < b_.footprints.size(); ++fi) fp_by_ref_.emplace(b_.footprints[fi].reference, static_cast<int>(fi));
  }
  for (const auto& rule : r_.custom) {
    Compiled c{&rule, nullptr, true};
    if (!rule.condition.empty()) {
      std::string err;
      c.cond = Condition::parse(rule.condition, *this, err);
      if (!c.cond) {
        c.valid = false;
        warnings_.push_back("rule '" + rule.name + "': cannot parse condition (" + err + "); rule ignored");
      }
    }
    rules_.push_back(std::move(c));
    for (const auto& k : rule.constraints)
      if (k.type == "clearance" && k.min) max_clearance_ = std::max(max_clearance_, *k.min);
  }
  static constexpr const char* kMinTypes[] = {"clearance",   "hole_clearance", "hole_to_hole",  "edge_clearance",
                                              "track_width", "via_diameter",   "annular_width", "hole_size"};
  static_assert(std::size(kMinTypes) == MinCount);
  for (const auto& c : rules_) {
    if (!c.valid) continue;
    model::LayerMask layers = 0;
    for (int l = 0; l < 64; ++l)
      if (layer_matches(c.rule->layer, l)) layers |= model::layer_bit(l);
    std::vector<char> by_class;
    if (c.cond)
      for (const auto& cls : r_.classes) by_class.push_back(c.cond->may_hold(cls.name));
    if (std::all_of(by_class.begin(), by_class.end(), [](char x) { return x; })) by_class.clear();
    for (std::size_t t = 0; t < MinCount; ++t) {
      const model::Constraint* k = nullptr;
      for (const auto& x : c.rule->constraints)
        if (x.type == kMinTypes[t] && x.min) k = &x;
      if (k) min_rules_[t].push_back({c.cond.get(), *k->min, layers, by_class});
    }
  }
  for (const auto& c : r_.classes) max_clearance_ = std::max(max_clearance_, c.clearance);
  // Per-net caches: net class and diff-pair partner (string matching is far too slow for inner loops).
  net_class_.resize(b_.nets.size());
  dp_partner_.assign(b_.nets.size(), 0);
  std::map<std::string, model::NetId> by_name;
  for (const auto& n : b_.nets) by_name[n.name] = n.id;
  for (const auto& n : b_.nets) {
    net_class_[static_cast<std::size_t>(n.id)] = &r_.class_for(n.name);
    if (n.name.size() < 2) continue;
    std::string other = n.name;
    char& last = other.back();
    if (last == 'P') last = 'N';
    else if (last == 'N') last = 'P';
    else if (last == '+') last = '-';
    else if (last == '-') last = '+';
    else continue;
    if (auto it = by_name.find(other); it != by_name.end()) dp_partner_[static_cast<std::size_t>(n.id)] = it->second;
  }
  for (const auto& rule : r_.custom)
    for (const auto& k : rule.constraints)
      if (k.type == "clearance") any_custom_clearance_ = true;
  for (const auto& p : b_.pads) max_clearance_ = std::max(max_clearance_, p.clearance);
  max_clearance_ = std::max({max_clearance_, r_.minimums.clearance, r_.minimums.copper_edge_clearance, r_.minimums.hole_clearance,
                             r_.minimums.hole_to_hole});
}

RuleEngine::~RuleEngine() = default;

void RuleEngine::use_zone_clearance_overrides() {
  zone_overrides_ = true;
  for (const auto& z : b_.zones)
    if (!z.rule_area) max_clearance_ = std::max(max_clearance_, z.clearance);
}

const model::NetClass& RuleEngine::netclass(model::NetId net) const {
  const auto i = static_cast<std::size_t>(net);
  if (i < net_class_.size()) return *net_class_[i];
  return r_.class_for(b_.nets[i].name);
}

bool RuleEngine::layer_matches(const std::string& sel, int layer) const {
  if (sel.empty() || layer < 0) return true;
  const int last = b_.copper_count() - 1;
  if (sel == "outer") return layer == 0 || layer == last;
  if (sel == "inner") return layer > 0 && layer < last;
  return b_.copper_index(sel) == layer;
}

std::optional<Coord> RuleEngine::custom_min(Min type, const CopperItem* a, const CopperItem* b, int layer) const {
  // Later rules take precedence: the last match decides, so look from the back.
  auto may_hold = [&](const MinRule& m, const CopperItem* x) {
    return !x || m.by_class.empty() || m.by_class[static_cast<std::size_t>(&netclass(*x) - r_.classes.data())];
  };
  const auto& rules = min_rules_[type];
  for (auto m = rules.rbegin(); m != rules.rend(); ++m) {
    if (layer >= 0 && !(m->layers & model::layer_bit(layer))) continue;
    if (!m->cond) return m->min;
    if (may_hold(*m, a)) {
      EvalCtx ctx{this, a, b, layer};
      if (m->cond->eval(ctx)) return m->min;
    }
    if (b && may_hold(*m, b)) {
      EvalCtx ctx2{this, b, a, layer};
      if (m->cond->eval(ctx2)) return m->min;
    }
  }
  return std::nullopt;
}

std::pair<std::optional<Coord>, std::optional<Coord>> RuleEngine::length_constraint(model::NetId net) const {
  std::pair<std::optional<Coord>, std::optional<Coord>> out;
  CopperItem probe;
  probe.kind = ItemKind::Track;
  probe.net = net;
  probe.layers = ~model::LayerMask{0};
  for (const auto& c : rules_) {
    if (!c.valid) continue;
    const model::Constraint* k = nullptr;
    for (const auto& x : c.rule->constraints)
      if (x.type == "length") k = &x;
    if (!k) continue;
    if (c.cond) {
      EvalCtx ctx{this, &probe, nullptr, -1};
      if (!c.cond->eval(ctx)) continue;
    }
    out = {k->min, k->max};  // later rules take precedence
  }
  return out;
}

std::optional<Coord> RuleEngine::skew_constraint(model::NetId net) const {
  std::optional<Coord> out;
  CopperItem probe;
  probe.kind = ItemKind::Track;
  probe.net = net;
  probe.layers = ~model::LayerMask{0};
  for (const auto& c : rules_) {
    if (!c.valid) continue;
    const model::Constraint* k = nullptr;
    for (const auto& x : c.rule->constraints)
      if (x.type == "skew" && x.max) k = &x;
    if (!k) continue;
    if (c.cond) {
      EvalCtx ctx{this, &probe, nullptr, -1};
      if (!c.cond->eval(ctx)) continue;
    }
    out = k->max;
  }
  return out;
}

std::optional<model::Constraint> RuleEngine::net_constraint(model::NetId net, const std::string& type) const {
  std::optional<model::Constraint> out;
  CopperItem probe;
  probe.kind = ItemKind::Track;
  probe.net = net;
  probe.layers = ~model::LayerMask{0};
  for (const auto& c : rules_) {
    if (!c.valid) continue;
    const model::Constraint* k = nullptr;
    for (const auto& x : c.rule->constraints)
      if (x.type == type) k = &x;
    if (!k) continue;
    if (c.cond) {
      EvalCtx ctx{this, &probe, nullptr, -1};
      if (!c.cond->eval(ctx)) continue;
    }
    out = *k;  // later rules take precedence
  }
  return out;
}

bool RuleEngine::net_specific_clearance() const {
  for (const auto& c : rules_) {
    // The text is enough: a property or function the engine evaluates appears verbatim in it.
    const std::string& t = c.rule->condition;
    if (!c.valid || (t.find("NetName") == std::string::npos && t.find("inDiffPair") == std::string::npos)) continue;
    for (const auto& k : c.rule->constraints)
      if (k.type == "clearance" || k.type == "hole_clearance" || k.type == "edge_clearance" || k.type == "hole_to_hole") return true;
  }
  return false;
}

bool RuleEngine::coupled_diff_pair(model::NetId a, model::NetId b) const {
  if (a == 0 || b == 0 || a == b) return false;
  if (static_cast<std::size_t>(a) < dp_partner_.size()) return dp_partner_[static_cast<std::size_t>(a)] == b;
  const std::string& x = b_.nets[static_cast<std::size_t>(a)].name;
  const std::string& y = b_.nets[static_cast<std::size_t>(b)].name;
  if (x.size() != y.size() || x.empty()) return false;
  const char cx = x.back(), cy = y.back();
  const bool pair = (cx == 'P' && cy == 'N') || (cx == 'N' && cy == 'P') || (cx == '+' && cy == '-') || (cx == '-' && cy == '+');
  return pair && x.compare(0, x.size() - 1, y, 0, y.size() - 1) == 0;
}

Coord RuleEngine::clearance(const CopperItem& a, const CopperItem& b, int layer) const {
  // Local overrides (pad, then footprint) win over net classes: KiCad uses the larger of the two items'
  // overrides when either has one (verified on tinytapeout-demo: U6 footprint 0.18 mm vs GND class 0.23 mm).
  auto local = [&](const CopperItem& it) -> Coord {
    if (it.kind != ItemKind::Pad) return -1;
    const auto& pad = b_.pads[static_cast<std::size_t>(it.index)];
    if (pad.clearance >= 0) return pad.clearance;
    return b_.footprints[static_cast<std::size_t>(pad.footprint)].clearance;
  };
  const Coord la = local(a), lb = local(b);
  Coord req = (la >= 0 || lb >= 0) ? std::max(la, lb) : std::max(netclass(a).clearance, netclass(b).clearance);
  // KiCad's implicit diff-pair rule: between the two halves of a coupled pair, the class's diff-pair gap
  // replaces the clearance (verified on CM5_MINIMA_3: PCIe P/N at 0.177 mm under a 0.2 mm class clearance).
  // Observed: the smaller of clearance and diff-pair gap applies (0.154 gap passes under a 0.2 class; 0.2 mm
  // pads pass under a 0.25 gap).
  if (la < 0 && lb < 0 && coupled_diff_pair(a.net, b.net)) req = std::min(req, netclass(a).diff_pair_gap);
  bool custom = false;
  if (any_custom_clearance_)
    if (auto c = custom_min(Clearance, &a, &b, layer)) req = *c, custom = true;
  // A copper zone's own clearance is a local clearance: without a pad override or a custom rule, the larger of
  // it and the net-class value applies (DRC_ENGINE::EvalRules; KiCad names it "zone clearance").
  if (zone_overrides_ && la < 0 && lb < 0 && !custom)
    for (const CopperItem* z : {&a, &b})
      if (z->kind == ItemKind::Zone) req = std::max(req, b_.zones[static_cast<std::size_t>(z->index)].clearance);
  // The board minimum is always a floor (multichannel_mixer: 0.3 mm minimum over a 0.2 mm class).
  return std::max(req, r_.minimums.clearance);
}

Coord RuleEngine::hole_clearance(const CopperItem* owner, const CopperItem& other, int layer) const {
  Coord req = r_.minimums.hole_clearance;
  if (auto c = custom_min(HoleClearance, owner, &other, layer)) req = std::max(req, *c);
  return req;
}

Coord RuleEngine::hole_to_hole(const CopperItem* a, const CopperItem* b) const {
  Coord req = r_.minimums.hole_to_hole;
  if (auto c = custom_min(HoleToHole, a, b, -1)) req = std::max(req, *c);
  return req;
}

Coord RuleEngine::edge_clearance(const CopperItem& a, int layer) const {
  Coord req = r_.minimums.copper_edge_clearance;
  if (auto c = custom_min(EdgeClearance, &a, nullptr, layer)) req = *c;
  return req;
}

std::pair<Coord, Coord> RuleEngine::track_width(const CopperItem& t, int layer) const {
  Coord mn = r_.minimums.track_width, mx = 0;
  if (auto c = custom_min(TrackWidth, &t, nullptr, layer)) mn = *c;
  return {mn, mx};
}

Coord RuleEngine::via_diameter_min(const CopperItem& v) const {
  Coord req = r_.minimums.via_diameter;
  if (auto c = custom_min(ViaDiameter, &v, nullptr, -1)) req = *c;
  return req;
}

Coord RuleEngine::annular_width_min(const CopperItem& v) const {
  Coord req = r_.minimums.via_annular_width;
  if (auto c = custom_min(AnnularWidth, &v, nullptr, -1)) req = *c;
  return req;
}

Coord RuleEngine::hole_size_min(const CopperItem* owner) const {
  Coord req = r_.minimums.through_hole_diameter;
  if (auto c = custom_min(HoleSize, owner, nullptr, -1)) req = *c;
  return req;
}

}  // namespace tmk::drc
