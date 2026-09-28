#include "../include/skyline.h"
#include <algorithm>
#include <cmath>

Skyline::Skyline() {}
Skyline::~Skyline() {}

double Skyline::uniform(double _a, double _b) {
  std::uniform_real_distribution<double> dist(_a, _b);
  return dist(rng);
}

void Skyline::generate(unsigned _seed, double _halfWidth, double _halfHeight) {
  nodes.clear();
  buildings.clear();
  settled = false;
  growthSteps = 0;
  ringPos.clear();
  rng.seed(_seed);
  halfWidth = _halfWidth;
  halfHeight = _halfHeight;
  double ground = -halfHeight;

  // per layer: count, height range, width range (far layers smaller)
  const int counts[layers] = {8, 6, 4};
  // tall enough that the crowns form a canopy at the top of the screen
  const double minH[layers] = {180, 190, 200};
  const double maxH[layers] = {210, 220, 230};
  const double minW[layers] = {20, 24, 28};
  const double maxW[layers] = {32, 38, 45};

  for (int layer = 0; layer < layers; ++layer) {
    // spread the towers over the width, no overlaps inside a layer
    double slot = 2 * halfWidth / counts[layer];
    for (int i = 0; i < counts[layer]; ++i) {
      double w = uniform(minW[layer], maxW[layer]);
      double h = uniform(minH[layer], maxH[layer]);
      double room = std::max(0.0, slot - w - 8) / 2;
      double cx = -halfWidth + (i + 0.5) * slot + uniform(-room, room);
      int style = std::uniform_int_distribution<int>(0, 4)(rng);
      addBuilding(outline(style, cx, w, h, ground), cx, ground, h, layer);
    }
  }
}

// corners of a futuristic tower, counter clockwise starting bottom left
std::vector<Vec2> Skyline::outline(int _style, double _cx, double _w,
                                   double _h, double _base) {
  double l = _cx - _w / 2, r = _cx + _w / 2;
  auto at = [&](double _x, double _y) { return Vec2(_x, _base + _y * _h); };
  std::vector<Vec2> right; // right half from bottom to top (mirrored later)

  switch (_style) {
  case 0: // setbacks with a chamfered crown
    right = {at(r, 0),
             at(r, 0.55),
             at(r - 0.1 * _w, 0.55),
             at(r - 0.1 * _w, 0.8),
             at(r - 0.22 * _w, 0.8),
             at(r - 0.22 * _w, 0.94),
             at(_cx + 0.08 * _w, 1)};
    break;
  case 1: { // tapering needle with a spire
    double spire = std::max(2.5, 0.08 * _w);
    right = {at(r, 0), at(_cx + 0.3 * _w, 0.86), at(_cx + spire, 0.88),
             at(_cx + spire, 1)};
    break;
  }
  case 2: // bullet: straight shaft with an elliptic top
    right = {at(r, 0), at(r, 0.75)};
    for (int k = 1; k <= 8; ++k) {
      double a = M_PI / 2 * k / 8;
      right.push_back(at(_cx + _w / 2 * std::cos(a), 0.75 + 0.25 * std::sin(a)));
    }
    right.pop_back(); // top point is shared with the mirrored half
    break;
  case 3: { // slanted roof with a sky garden notch (asymmetric)
    double notch = std::max(5.0, 0.03 * _h);
    return {at(l, 0),
            at(r, 0),
            at(r, 0.6),
            at(r - 0.3 * _w, 0.6),
            at(r - 0.3 * _w, 0.6 + notch / _h),
            at(r, 0.6 + notch / _h),
            at(r, 0.88),
            at(l, 1)};
  }
  default: { // twin prongs with a gap
    double gap = std::max(6.0, 0.25 * _w);
    return {at(l, 0),
            at(r, 0),
            at(r, 1),
            at(_cx + gap / 2, 1),
            at(_cx + gap / 2, 0.8),
            at(_cx - gap / 2, 0.8),
            at(_cx - gap / 2, 0.93),
            at(l, 0.93)};
  }
  }

  // mirror the right half to the left side
  std::vector<Vec2> corners = {at(l, 0)};
  corners.insert(corners.end(), right.begin(), right.end());
  for (int k = right.size() - 1; k >= 1; --k)
    corners.push_back({2 * _cx - right[k].x, right[k].y});
  if (std::abs(right.back().x - _cx) < 1e-9)
    corners.pop_back(); // symmetric apex only once
  return corners;
}

// resample the corners to nodes with the rest edge length
void Skyline::addBuilding(const std::vector<Vec2> &_corners, double _cx,
                          double _base, double _h, int _layer) {
  Building b;
  b.cx = _cx;
  b.base = _base;
  b.height = _h;
  b.layer = _layer;
  int index = buildings.size();
  for (size_t k = 0; k < _corners.size(); ++k) {
    Vec2 a = _corners[k];
    Vec2 c = _corners[(k + 1) % _corners.size()];
    int n = std::max(1, (int)std::round((c - a).length() / edge));
    for (int s = 0; s < n; ++s) {
      nodes.push_back({lerp(a, c, (double)s / n), index, _layer, edge, 0});
      b.ring.push_back(nodes.size() - 1);
    }
  }
  b.maxNodes = maxGrowth * b.ring.size();
  buildings.push_back(b);
  ringPos.resize(nodes.size());
  for (size_t k = 0; k < b.ring.size(); ++k)
    ringPos[b.ring[k]] = k;
  seedTips(buildings.back());
}

// main branches: from the roof and from the facades above the growth start
void Skyline::seedTips(Building &_b) {
  int n = _b.ring.size();
  double top = _b.base + _b.height;
  std::vector<int> roof, left, right;
  for (int k = 0; k < n; ++k) {
    Vec2 p = nodes[_b.ring[k]].pos;
    Vec2 normal = outwardNormal(_b, k);
    if (growthMask(_b, p) < 0.3)
      continue;
    if (p.y > top - 0.08 * _b.height && normal.y > 0.3)
      roof.push_back(k);
    else if (normal.x > 0.7)
      right.push_back(k);
    else if (normal.x < -0.7)
      left.push_back(k);
  }

  auto addTip = [&](int _k, const Vec2 &_heading, double _budget) {
    _b.tips.push_back({_b.ring[_k], _heading.normalized(), _budget, 0});
  };
  // roof: spread evenly over the width
  int count = std::clamp((int)std::round(_b.height / 60), 2, 4);
  for (int t = 0; t < count && !roof.empty(); ++t) {
    int k = roof[(t * 2 + 1) * roof.size() / (2 * count)];
    Vec2 heading = growthDir(_b, nodes[_b.ring[k]].pos).rotated(uniform(-0.3, 0.3));
    addTip(k, heading, uniform(minLimb, maxLimb));
  }
  // facades: shorter limbs, reaching out and up
  for (auto *side : {&left, &right}) {
    for (int t = 0; t < facadeTips && !side->empty(); ++t) {
      int k = (*side)[std::uniform_int_distribution<int>(0, side->size() - 1)(rng)];
      Vec2 heading = (outwardNormal(_b, k) + Vec2(0, 1.2)).rotated(uniform(-0.2, 0.2));
      addTip(k, heading, 0.5 * uniform(minLimb, maxLimb));
    }
  }
}

// 0 below the growth start (unchanged city), rising to 1 above it
double Skyline::growthMask(const Building &, const Vec2 &_p) const {
  double t = std::clamp((_p.y - growthStart) / growthRamp, 0.0, 1.0);
  return t * t * (3 - 2 * t);
}

// preferred growth direction: up in the middle, up and out at the sides
Vec2 Skyline::growthDir(const Building &_b, const Vec2 &_p) const {
  double side = std::clamp((_p.x - _b.cx) / 20, -1.0, 1.0);
  return Vec2(0.8 * side, 1).normalized();
}

Vec2 Skyline::outwardNormal(const Building &_b, int _k) const {
  int n = _b.ring.size();
  Vec2 t = nodes[_b.ring[(_k + 1) % n]].pos - nodes[_b.ring[(_k + n - 1) % n]].pos;
  return Vec2(t.y, -t.x).normalized(); // ring is counter clockwise
}

// 1 up to the top of the tower, shrinking towards the ends of the branches
double Skyline::detailScale(const Building &_b, const Vec2 &_p) const {
  double t = (_p.y - (_b.base + 0.9 * _b.height)) / crownDepth;
  return 1 - taper * std::clamp(t, 0.0, 1.0);
}

// time profile of the growth: quick ramp up to a burst, then easing off
double Skyline::growthBoost() const {
  double t = growthSteps / stepsPerSecond;
  return std::min(1.0, t / rampTime) *
         (calm + (burst - calm) * std::exp(-t / decayTime));
}

int Skyline::hashCell(const Vec2 &_p) const {
  int c = std::clamp((int)((_p.x - hashMinX) / repelRadius), 0, hashCols - 1);
  int r = std::clamp((int)((_p.y - hashMinY) / repelRadius), 0, hashRows - 1);
  return r * hashCols + c;
}

void Skyline::buildHash() {
  hashMinX = -halfWidth - 100;
  hashMinY = -halfHeight - 10;
  hashCols = (2 * halfWidth + 200) / repelRadius + 1;
  hashRows = (2 * halfHeight + 200) / repelRadius + 1;
  head.assign(hashCols * hashRows, -1);
  next.assign(nodes.size(), -1);
  for (int i = 0; i < (int)nodes.size(); ++i) {
    int c = hashCell(nodes[i].pos);
    next[i] = head[c];
    head[c] = i;
  }
}

void Skyline::step() {
  if (settled)
    return;
  buildHash();
  moves.assign(nodes.size(), {0, 0});
  // roughness follows the growth burst, the outline calms down afterwards
  double rough = jitter * growthBoost() / burst;
  int hardenSteps = hardenTime * stepsPerSecond;
  std::vector<char> isTip(nodes.size(), 0);
  for (const Building &b : buildings)
    for (const GrowthTip &tip : b.tips)
      isTip[tip.node] = 1;

  for (const Building &b : buildings) {
    int n = b.ring.size();
    for (int k = 0; k < n; ++k) {
      int i = b.ring[k];
      int prev = b.ring[(k + n - 1) % n];
      int nxt = b.ring[(k + 1) % n];
      Vec2 x = nodes[i].pos;
      double g = growthMask(b, x);
      if (g <= 0)
        continue; // frozen skyscraper
      if (!isTip[i] && growthSteps - nodes[i].born > hardenSteps)
        continue; // grown wood keeps its shape

      // finer detail higher up in the crown -> branches taper into spikes
      double radius = repelRadius * detailScale(b, x);

      Vec2 f;
      if (!isTip[i]) {
        // springs to the neighbours
        for (int q : {prev, nxt}) {
          Vec2 d = nodes[q].pos - x;
          double len = d.length();
          double rest = (nodes[i].rest + nodes[q].rest) / 2;
          if (len > 1e-9)
            f += d / len * ((len - rest) * kSpring);
        }
        // smoothing towards the neighbours' midpoint
        f += ((nodes[prev].pos + nodes[nxt].pos) / 2 - x) * kSmooth;
        // roughness
        f += Vec2(uniform(-1, 1), uniform(-1, 1)) * rough;
      }

      // repulsion from all other nodes of the same layer
      int c = hashCell(x);
      int cc = c % hashCols, cr = c / hashCols;
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          int col = cc + dc, row = cr + dr;
          if (col < 0 || col >= hashCols || row < 0 || row >= hashRows)
            continue;
          for (int j = head[row * hashCols + col]; j >= 0; j = next[j]) {
            if (j == i || j == prev || j == nxt || nodes[j].layer != b.layer)
              continue;
            Vec2 d = x - nodes[j].pos;
            double dist = d.length();
            if (dist < radius && dist > 1e-9)
              f += d / dist * ((radius - dist) * kRepel);
          }
        }
      }
      moves[i] = isTip[i] ? f : (f * g).clampedLength(maxStep);
    }
  }

  // branch tips pull ahead along their heading
  double boost = growthBoost();
  for (Building &b : buildings)
    moveTips(b, boost);

  before.resize(nodes.size());
  double maxMove = 0;
  for (size_t i = 0; i < nodes.size(); ++i) {
    before[i] = nodes[i].pos;
    nodes[i].pos += moves[i];
    maxMove = std::max(maxMove, moves[i].length());
  }

  split();
  bool growing = false;
  for (Building &b : buildings) {
    updateTips(b);
    growing = growing || !b.tips.empty();
  }
  // done growing and at rest -> stop simulating
  settled = !growing && maxMove < settleMove;
  growthSteps++;
}

void Skyline::moveTips(Building &_b, double _boost) {
  for (const GrowthTip &tip : _b.tips) {
    Vec2 f = moves[tip.node] + tip.heading * (tipSpeed * _boost);
    moves[tip.node] = f.clampedLength(maxTipStep);
  }
}

// progress, kinks, side branches and end of growth of the tips
void Skyline::updateTips(Building &_b) {
  std::vector<GrowthTip> spawned;
  for (GrowthTip &tip : _b.tips) {
    Vec2 moved = nodes[tip.node].pos - before[tip.node];
    double grown = std::max(0.0, moved.dot(tip.heading));
    tip.budget -= grown;
    tip.stuck = grown < 0.3 * tipSpeed * growthBoost() ? tip.stuck + 1 : 0;

    // dead wood: straight pieces with sudden kinks, slowly bending towards
    // the growth direction, never hanging down
    if (uniform(0, 1) < grown / kinkLength) {
      double angle = uniform(minKink, maxKink);
      tip.heading = tip.heading.rotated(uniform(0, 1) < 0.5 ? angle : -angle);
    }
    tip.heading += growthDir(_b, nodes[tip.node].pos) * tropism;
    tip.heading.y = std::max(tip.heading.y, -0.1);
    tip.heading = tip.heading.normalized();

    // occasional side branch, the tip itself keeps growing
    if (tip.depth < maxDepth && uniform(0, 1) < grown / branchLength)
      spawned.push_back(tip);
  }
  // finished, blocked or out of room
  bool full = (int)_b.ring.size() >= _b.maxNodes;
  _b.tips.erase(std::remove_if(_b.tips.begin(), _b.tips.end(),
                               [&](const GrowthTip &_t) {
                                 return full || _t.budget <= 0 || _t.stuck > 30;
                               }),
                _b.tips.end());
  for (const GrowthTip &parent : spawned)
    branch(_b, parent);
}

// side branch from the flank of a limb a few nodes behind its tip,
// turned away to the same side
void Skyline::branch(Building &_b, const GrowthTip &_parent) {
  if ((int)_b.tips.size() >= maxTips)
    return;
  int n = _b.ring.size();
  int side = uniform(0, 1) < 0.5 ? 1 : -1; // +1: left flank (ccw ring)
  int back = std::uniform_int_distribution<int>(3, 8)(rng);
  int k = ((ringPos[_parent.node] + side * back) % n + n) % n;
  int node = _b.ring[k];
  if (growthMask(_b, nodes[node].pos) < 0.5)
    return;
  for (const GrowthTip &tip : _b.tips)
    if (tip.node == node)
      return;
  Vec2 heading = _parent.heading.rotated(side * uniform(0.5, 1.0));
  _b.tips.push_back({node, heading, _parent.budget * uniform(0.4, 0.7),
                     _parent.depth + 1});
}

// split edges which got too long (behind the tips), keeps node positions
void Skyline::split() {
  for (Building &b : buildings) {
    int n = b.ring.size();
    if (n >= b.maxNodes)
      continue; // hard limit, keeps the cost bounded
    std::vector<int> ring;
    ring.reserve(n + 16);
    for (int k = 0; k < n; ++k) {
      int i = b.ring[k];
      int j = b.ring[(k + 1) % n];
      ring.push_back(i);
      Vec2 mid = (nodes[i].pos + nodes[j].pos) / 2;
      if (growthMask(b, mid) <= 0)
        continue;
      if ((nodes[j].pos - nodes[i].pos).length() >
          splitLength * detailScale(b, mid)) {
        nodes.push_back({mid, nodes[i].building, b.layer,
                         edge * detailScale(b, mid), growthSteps});
        before.push_back(mid);
        ring.push_back(nodes.size() - 1);
      }
    }
    b.ring = ring;
  }
  ringPos.resize(nodes.size());
  for (const Building &b : buildings)
    for (size_t k = 0; k < b.ring.size(); ++k)
      ringPos[b.ring[k]] = k;
}

// translucent like the canopy: gruv-light with low alpha, soft edges from
// four slightly offset passes (inside they add up, at the edges they fade)
void Skyline::draw(UI &_ui, bool _debug) {
  const double alpha[layers] = {20, 32, 45}; // far layers fade out
  const double blur = 1.5 / _ui.scale;       // 1.5 px offset per pass
  const Vec2 offsets[4] = {{-blur, -blur}, {blur, -blur}, {-blur, blur},
                           {blur, blur}};
  std::vector<Vec2> points;

  for (int layer = 0; layer < layers; ++layer) {
    // alpha of one pass so that four passes give the layer's alpha
    double a = alpha[layer] / 255;
    _ui.setAlpha(255 * (1 - std::pow(1 - a, 0.25)));
    for (const Building &b : buildings) {
      if (b.layer != layer)
        continue;
      for (const Vec2 &o : offsets) {
        points.clear();
        for (int i : b.ring)
          points.push_back(nodes[i].pos + o);
        _ui.scanPolygon(points, [&](int _row, float _x0, float _x1) {
          _ui.drawSpan(_row, _x0, _x1);
        });
      }
    }
  }

  if (!_debug)
    return;
  for (const Building &b : buildings) {
    for (int i : b.ring) {
      _ui.setAlpha(60 + 195 * growthMask(b, nodes[i].pos));
      _ui.fillCircle(nodes[i].pos, 1.2 / _ui.scale);
    }
    _ui.setAlpha(255);
    for (const GrowthTip &tip : b.tips)
      _ui.drawCircle(nodes[tip.node].pos, 5 / _ui.scale);
  }
}
