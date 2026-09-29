#ifndef CITY_H
#define CITY_H

#pragma once
#include "UI.h"
#include "cable_network.h"
#include "vec2.h"
#include <memory>
#include <vector>

// the city the cables hang from, interchangeable (see makeCity)
class City {
public:
  virtual ~City() = default;

  // world extents in meters, picks the cable anchors
  virtual void generate(unsigned, double, double, int) = 0;
  // anchor positions in world meters, same order as long as the city lives
  virtual const std::vector<Vec2> &getAnchors() const = 0;
  // advance by _dt, _focus: pod position; true if the anchors moved
  virtual bool update(double, const Vec2 &) = 0;

  virtual void drawBack(UI &) = 0; // behind the cables and the pod
  virtual void drawFront(UI &) {} // in front of them
  virtual CableStyle cableStyle() const { return {}; }
};

enum class CityType {
  GROWING,  // skyline growing into a canopy of dead trees
  PARALLAX, // static skyline in three layers with parallax
};

std::unique_ptr<City> makeCity(CityType);

#endif
