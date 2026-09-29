#include "../include/growing_city.h"

void GrowingCity::generate(unsigned _seed, double _halfWidth,
                           double _halfHeight, int _anchorCount) {
  skyline.setWorldScale(scale);
  skyline.generate(_seed, _halfWidth / scale, _halfHeight / scale);
  anchorNodes = skyline.pickAnchors(_anchorCount, 1);
  updateAnchors();
  time = 0;
  growthDue = 0;
}

void GrowingCity::updateAnchors() {
  anchors.clear();
  for (int node : anchorNodes)
    anchors.push_back(skyline.nodePosition(node));
}

// the city grows, the cable anchors move along with it
bool GrowingCity::update(double _dt, const Vec2 &) {
  time += _dt;
  if (time <= growthDelay || skyline.getSettled())
    return false;
  growthDue += growthSpeed;
  for (; growthDue >= 1; growthDue -= 1)
    skyline.step();
  updateAnchors();
  return true;
}

void GrowingCity::drawBack(UI &_ui) { skyline.draw(_ui, false); }
