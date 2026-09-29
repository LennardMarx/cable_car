#ifndef GROWING_CITY_H
#define GROWING_CITY_H

#pragma once
#include "city.h"
#include "skyline.h"

// the growing skyline, drawn smaller; the cables hang from its nodes and
// move with the growth
class GrowingCity : public City {
public:
  void generate(unsigned, double, double, int) override;
  const std::vector<Vec2> &getAnchors() const override { return anchors; }
  bool update(double, const Vec2 &) override;
  void drawBack(UI &) override;

private:
  void updateAnchors();

  Skyline skyline;
  std::vector<int> anchorNodes; // skyline node of every cable anchor
  std::vector<Vec2> anchors;
  double time = 0;
  double growthDue = 0; // fractional growth steps not yet done

  const double scale = 0.15;      // world meters per skyline meter
  const double growthDelay = 0.5; // s showing the city before it grows
  const double growthSpeed = 0.9; // growth steps per time step (demo: 3)
};

#endif
