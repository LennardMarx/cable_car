#ifndef PARALLAX_CITY_H
#define PARALLAX_CITY_H

#pragma once
#include "city.h"
#include <random>

// one depth layer of the static skyline
struct CityLayer {
  std::vector<std::vector<Vec2>> shapes; // towers and bridges (world meters)
  std::vector<Vec2> holes, lit;          // window quads, 4 corners each
};

// static skyline in three layers: far towers showing at the top, the middle
// layer the cables hang from (and the pod moves in), near rooftops at the
// bottom. The view shifts sideways with the pod (parallax); the cables and
// the pod are drawn shifted with the middle layer, their physics is not
class ParallaxCity : public City {
public:
  void generate(unsigned, double, double, int) override;
  const std::vector<Vec2> &getAnchors() const override { return anchors; }
  bool update(double, const Vec2 &) override;
  void drawBack(UI &) override;
  void drawFront(UI &) override;
  CableStyle cableStyle() const override;
  Vec2 viewOffset() const override { return {shift[1] * view, 0}; }

  static inline const int layers = 3; // 0 far, 1 middle, 2 near

private:
  void addTowers(int);
  void addBridges();
  void addWindows(int, const std::vector<Vec2> &, double, double);
  void pickAnchors(int);
  void drawLayer(UI &, int);
  static std::vector<double> crossings(const std::vector<Vec2> &, double);
  double uniform(double, double);

  CityLayer cityLayers[layers];
  std::vector<std::vector<Vec2>> middleTowers; // for bridges and anchors
  std::vector<Vec2> anchors;
  std::vector<Vec2> bridgeAnchors;
  std::vector<Vec2> quads; // shifted window quads (reused)

  std::mt19937 rng;
  double halfWidth = 30, halfHeight = 20;
  double view = 0; // smoothed pod position across the screen, -1 ... 1

  // per layer, heights as a share of the screen height from its bottom,
  // widths as a share of the screen width
  const int counts[layers] = {11, 7, 6};
  const double minWidth[layers] = {0.045, 0.09, 0.13};
  const double maxWidth[layers] = {0.075, 0.13, 0.2};
  const double minTop[layers] = {0.86, 0.72, 0.12}; // background shows at the
  const double maxTop[layers] = {0.97, 0.84, 0.2}; // top, near at the bottom
  const double baseDepth[layers] = {0, 0, 0.8}; // ground below the screen
  // detail size: world meters per meter of the tower shapes (far: smaller)
  const double detail[layers] = {0.09, 0.15, 0.28};
  // sideways shift [m] with the pod at the edge of the screen: the far layer
  // follows it, the middle and near ones move against it
  const double shift[layers] = {1.0, -1.2, -4.5};
  const double smoothing = 0.6; // s time constant of the view
  // intensity: the near layer strongest, the far one faintest
  const double alpha[layers] = {26, 70, 130};
  const uint8_t holeAlpha[layers] = {90, 150, 200};
  const uint8_t litAlpha[layers] = {35, 70, 120};

  // bridges between neighbouring middle towers
  const double bridgeLow = 0.35; // share of the screen height
  const double minBridge = 0.5, maxBridge = 0.9; // thickness [m]
  // anchors: facades and roofs of the middle towers, undersides of bridges
  const double anchorLow = 0.45; // share of the screen height
  const double anchorPitch = 1.2; // m between candidates
  const double bridgeShare = 0.4; // of the anchors on bridges

  // windows, in meters of the tower shapes (scaled by the layer's detail)
  const double floorPitch = 5;
  const double windowWidth = 2.2, windowHeight = 2.6;
  const double windowPitch = 4;
  const double wallMargin = 2;
  const double litShare = 0.08;
};

#endif
