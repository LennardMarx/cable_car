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
  rng.seed(_seed);
  halfWidth = _halfWidth;
  halfHeight = _halfHeight;
  double ground = -halfHeight;

  // per layer: count, height range, width range (far layers smaller)
  const int counts[layers] = {8, 6, 4};
  const double minH[layers] = {90, 110, 130};
  const double maxH[layers] = {150, 175, 200};
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
      nodes.push_back({lerp(a, c, (double)s / n), index, _layer});
      b.ring.push_back(nodes.size() - 1);
    }
  }
  b.maxNodes = maxGrowth * b.ring.size();
  buildings.push_back(b);
}

// 0 on the lower 60 % (unchanged), rising to 1 towards the top
double Skyline::growthMask(const Building &_b, const Vec2 &_p) const {
  double t = (_p.y - (_b.base + 0.6 * _b.height)) / (0.3 * _b.height);
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3 - 2 * t);
}

// preferred growth direction: up in the middle, up and out at the sides
Vec2 Skyline::growthDir(const Building &_b, const Vec2 &_p) const {
  double side = std::clamp((_p.x - _b.cx) / 20, -1.0, 1.0);
  return Vec2(1.4 * side, 1).normalized();
}

Vec2 Skyline::outwardNormal(const Building &_b, int _k) const {
  int n = _b.ring.size();
  Vec2 t = nodes[_b.ring[(_k + 1) % n]].pos - nodes[_b.ring[(_k + n - 1) % n]].pos;
  return Vec2(t.y, -t.x).normalized(); // ring is counter clockwise
}

// how much node _k is the tip of a growing branch: facing the crown
// direction and convex (turning angle of the outline), flat parts grow slowly
double Skyline::tipFactor(const Building &_b, int _k) const {
  int n = _b.ring.size();
  Vec2 p = nodes[_b.ring[(_k + n - 1) % n]].pos;
  Vec2 x = nodes[_b.ring[_k]].pos;
  Vec2 q = nodes[_b.ring[(_k + 1) % n]].pos;
  double turn = std::atan2((x - p).cross(q - x), (x - p).dot(q - x));
  double convex = std::clamp(turn / tipAngle, 0.0, 1.0);
  double facing = std::max(0.0, outwardNormal(_b, _k).dot(growthDir(_b, x)));
  return facing * facing * (0.05 + convex);
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

      Vec2 f;
      // springs to the neighbours
      for (int q : {prev, nxt}) {
        Vec2 d = nodes[q].pos - x;
        double len = d.length();
        if (len > 1e-9)
          f += d / len * ((len - edge) * kSpring);
      }
      // smoothing towards the neighbours' midpoint
      f += ((nodes[prev].pos + nodes[nxt].pos) / 2 - x) * kSmooth;

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
            if (dist < repelRadius && dist > 1e-9)
              f += d / dist * ((repelRadius - dist) * kRepel);
          }
        }
      }

      // growing tips push outwards
      f += outwardNormal(b, k) * (kPush * tipFactor(b, k));

      moves[i] = (f * g).clampedLength(maxStep);
    }
  }

  double maxMove = 0;
  for (size_t i = 0; i < nodes.size(); ++i) {
    nodes[i].pos += moves[i];
    maxMove = std::max(maxMove, moves[i].length());
  }

  // done growing and at rest -> stop simulating
  bool full = true;
  for (const Building &b : buildings)
    full = full && (int)b.ring.size() >= b.maxNodes * 0.97;
  settled = full && maxMove < settleMove;

  grow();
}

// split long edges and insert new nodes at the growing tips,
// growth slows down towards the node limit
void Skyline::grow() {
  for (Building &b : buildings) {
    int n = b.ring.size();
    double slowdown = std::max(0.0, 1.0 - (double)n / b.maxNodes);
    std::vector<int> ring;
    ring.reserve(n + 16);
    for (int k = 0; k < n; ++k) {
      int i = b.ring[k];
      int j = b.ring[(k + 1) % n];
      ring.push_back(i);
      Vec2 mid = (nodes[i].pos + nodes[j].pos) / 2;
      double g = growthMask(b, mid);
      if (g <= 0)
        continue;
      double tip = (tipFactor(b, k) + tipFactor(b, (k + 1) % n)) / 2;
      if ((nodes[j].pos - nodes[i].pos).length() > splitLength ||
          uniform(0, 1) < growthRate * g * tip * slowdown) {
        nodes.push_back({mid, nodes[i].building, b.layer});
        ring.push_back(nodes.size() - 1);
      }
    }
    b.ring = ring;
  }
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
  }
}
