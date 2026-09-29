#include "../include/parallax_city.h"
#include "../include/skyline.h"
#include <algorithm>
#include <cmath>

double ParallaxCity::uniform(double _a, double _b) {
  std::uniform_real_distribution<double> dist(_a, _b);
  return dist(rng);
}

void ParallaxCity::generate(unsigned _seed, double _halfWidth,
                            double _halfHeight, int _anchorCount) {
  rng.seed(_seed);
  halfWidth = _halfWidth;
  halfHeight = _halfHeight;
  view = 0;
  middleTowers.clear();
  bridgeAnchors.clear();
  for (int layer = 0; layer < layers; ++layer) {
    cityLayers[layer] = {};
    addTowers(layer);
  }
  addBridges();
  pickAnchors(_anchorCount);
}

// x of the crossings of a closed polygon with the height _y, sorted (pairs
// enclose its inside)
std::vector<double> ParallaxCity::crossings(const std::vector<Vec2> &_poly,
                                            double _y) {
  std::vector<double> xs;
  for (size_t k = 0; k < _poly.size(); ++k) {
    Vec2 a = _poly[k], b = _poly[(k + 1) % _poly.size()];
    if ((a.y <= _y) == (b.y <= _y))
      continue;
    xs.push_back(a.x + (_y - a.y) / (b.y - a.y) * (b.x - a.x));
  }
  std::sort(xs.begin(), xs.end());
  return xs;
}

// towers spread over the width the layer can be shifted by, no overlaps
void ParallaxCity::addTowers(int _layer) {
  double span = halfWidth + std::abs(shift[_layer]) + 2;
  double slot = 2 * span / counts[_layer];
  double screenHeight = 2 * halfHeight;
  double base = -halfHeight - baseDepth[_layer] * screenHeight;
  double s = detail[_layer];
  for (int i = 0; i < counts[_layer]; ++i) {
    double w = uniform(minWidth[_layer], maxWidth[_layer]) * 2 * halfWidth;
    double top = -halfHeight + uniform(minTop[_layer], maxTop[_layer]) * screenHeight;
    double room = std::max(0.0, slot - w - 0.5) / 2;
    double cx = -span + (i + 0.5) * slot + uniform(-room, room);
    int style = std::uniform_int_distribution<int>(0, Skyline::styles - 1)(rng);
    // the shapes are made for meters of the growing skyline
    std::vector<Vec2> tower =
        Skyline::outline(style, cx / s, w / s, (top - base) / s, base / s);
    for (Vec2 &p : tower)
      p *= s;
    addWindows(_layer, tower, base, top);
    cityLayers[_layer].shapes.push_back(tower);
    if (_layer == 1)
      middleTowers.push_back(tower);
  }
}

// grid of windows per floor, only where the floor above contains them
void ParallaxCity::addWindows(int _layer, const std::vector<Vec2> &_tower,
                              double _base, double _top) {
  double s = detail[_layer];
  double w = windowWidth * s, h = windowHeight * s, pitch = windowPitch * s;
  double margin = wallMargin * s;
  CityLayer &l = cityLayers[_layer];
  for (double y = _base + floorPitch * s; y + h < _top - margin;
       y += floorPitch * s) {
    if (y + h < -halfHeight || y > halfHeight)
      continue; // off screen
    std::vector<double> bottom = crossings(_tower, y);
    std::vector<double> upper = crossings(_tower, y + h);
    for (size_t k = 0; k + 1 < bottom.size(); k += 2) {
      double xl = bottom[k], xr = bottom[k + 1];
      int columns = (xr - xl - 2 * margin + pitch - w) / pitch;
      if (columns < 1)
        continue;
      // centered in the floor
      double start = (xl + xr) / 2 - (columns * pitch - (pitch - w)) / 2;
      for (int col = 0; col < columns; ++col) {
        double x0 = start + col * pitch, x1 = x0 + w;
        bool inside = false;
        for (size_t u = 0; u + 1 < upper.size() && !inside; u += 2)
          inside = upper[u] <= x0 && x1 <= upper[u + 1];
        if (!inside)
          continue;
        std::vector<Vec2> &quads = uniform(0, 1) < litShare ? l.lit : l.holes;
        quads.insert(quads.end(), {{x0, y}, {x1, y}, {x1, y + h}, {x0, y + h}});
      }
    }
  }
}

// sky bridges between neighbouring middle towers, the cables hang from
// their undersides as well
void ParallaxCity::addBridges() {
  double low = -halfHeight + bridgeLow * 2 * halfHeight;
  auto roof = [](const std::vector<Vec2> &_tower) {
    double y = -1e9;
    for (const Vec2 &p : _tower)
      y = std::max(y, p.y);
    return y;
  };
  for (size_t i = 0; i + 1 < middleTowers.size(); ++i) {
    const std::vector<Vec2> &left = middleTowers[i], &right = middleTowers[i + 1];
    double top = std::min(roof(left), roof(right));
    // well below the lower roof (setbacks and crowns)
    double high = low + 0.75 * (top - low);
    int count = std::uniform_int_distribution<int>(1, 2)(rng);
    for (int b = 0; b < count && high > low; ++b) {
      double y = uniform(low, high);
      double thickness = uniform(minBridge, maxBridge);
      // inner walls at the bottom and the top of the bridge (exact fit)
      std::vector<double> l0 = crossings(left, y), l1 = crossings(left, y + thickness);
      std::vector<double> r0 = crossings(right, y), r1 = crossings(right, y + thickness);
      if (l0.empty() || l1.empty() || r0.empty() || r1.empty())
        continue;
      double xl = l0.back(), xr = r0.front();
      if (xr - xl < 1)
        continue;
      cityLayers[1].shapes.push_back(
          {{xl, y}, {xr, y}, {r1.front(), y + thickness}, {l1.back(), y + thickness}});
      for (double x = xl + anchorPitch / 2; x < xr - anchorPitch / 2; x += anchorPitch)
        bridgeAnchors.push_back({x, y});
    }
  }
}

// points on the outlines of the middle towers (upper part of the screen)
// and under the bridges
void ParallaxCity::pickAnchors(int _count) {
  double low = -halfHeight + anchorLow * 2 * halfHeight;
  auto onScreen = [&](const Vec2 &_p) {
    return _p.y > low && _p.y < halfHeight - 0.5 && std::abs(_p.x) < halfWidth - 0.5;
  };
  std::vector<Vec2> walls, bridges;
  for (const std::vector<Vec2> &tower : middleTowers) {
    for (size_t k = 0; k < tower.size(); ++k) {
      Vec2 a = tower[k], b = tower[(k + 1) % tower.size()];
      int n = std::max(1, (int)std::round((b - a).length() / anchorPitch));
      for (int j = 0; j < n; ++j) {
        Vec2 p = lerp(a, b, (double)j / n);
        if (onScreen(p))
          walls.push_back(p);
      }
    }
  }
  for (const Vec2 &p : bridgeAnchors)
    if (onScreen(p))
      bridges.push_back(p);

  std::shuffle(walls.begin(), walls.end(), rng);
  std::shuffle(bridges.begin(), bridges.end(), rng);
  anchors.clear();
  int fromBridges = std::min<int>(bridges.size(), bridgeShare * _count);
  anchors.insert(anchors.end(), bridges.begin(), bridges.begin() + fromBridges);
  int fromWalls = std::min<int>(walls.size(), _count - fromBridges);
  anchors.insert(anchors.end(), walls.begin(), walls.begin() + fromWalls);
}

// the view follows the pod across the screen, smoothly
bool ParallaxCity::update(double _dt, const Vec2 &_focus) {
  double goal = std::clamp(_focus.x / halfWidth, -1.0, 1.0);
  view += (goal - view) * (1 - std::exp(-_dt / smoothing));
  return false; // the middle layer stays in place with its anchors
}

// translucent like the growing skyline: soft edges from four slightly
// offset passes, windows as holes (background color) and a few lit ones
void ParallaxCity::drawLayer(UI &_ui, int _layer) {
  const CityLayer &l = cityLayers[_layer];
  Vec2 offset(shift[_layer] * view, 0);
  const double blur = 1.5 / _ui.scale; // 1.5 px offset per pass
  const Vec2 passes[4] = {{-blur, -blur}, {blur, -blur}, {-blur, blur},
                          {blur, blur}};
  // alpha of one pass so that four passes give the layer's alpha
  _ui.setAlpha(255 * (1 - std::pow(1 - alpha[_layer] / 255, 0.25)));
  std::vector<Vec2> points;
  for (const std::vector<Vec2> &shape : l.shapes) {
    for (const Vec2 &o : passes) {
      points.clear();
      for (const Vec2 &p : shape)
        points.push_back(p + offset + o);
      _ui.scanPolygon(points, [&](int _row, float _x0, float _x1) {
        _ui.drawSpan(_row, _x0, _x1);
      });
    }
  }

  auto shifted = [&](const std::vector<Vec2> &_quads) -> const std::vector<Vec2> & {
    quads.clear();
    for (const Vec2 &p : _quads)
      quads.push_back(p + offset);
    return quads;
  };
  _ui.setDrawColor(69, 133, 136, holeAlpha[_layer]); // gruv-blue
  _ui.fillQuads(shifted(l.holes));
  _ui.setAlpha(litAlpha[_layer]);
  _ui.fillQuads(shifted(l.lit));
}

void ParallaxCity::drawBack(UI &_ui) {
  drawLayer(_ui, 0);
  drawLayer(_ui, 1);
}

void ParallaxCity::drawFront(UI &_ui) { drawLayer(_ui, 2); }

// the cables belong to the middle layer: same intensity, no mounts
CableStyle ParallaxCity::cableStyle() const {
  return {alpha[1] - 10, 20, false};
}
