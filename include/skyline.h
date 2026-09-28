#ifndef SKYLINE_H
#define SKYLINE_H

#pragma once
#include "UI.h"
#include "vec2.h"
#include <random>
#include <vector>

// node of a building outline, never removed (stable index for attachments)
struct GrowthNode {
  Vec2 pos;
  int building;
  int layer;
};

struct Building {
  std::vector<int> ring; // node indices along the outline, counter clockwise
  double cx;             // center line
  double base;           // ground level
  double height;         // original height
  int maxNodes;          // growth stops here
  int layer;             // 0 far, 1 middle, 2 near
};

// skyscrapers whose tops morph into organic, tree-like crowns
// by differential growth, the lower part stays unchanged
class Skyline {
public:
  Skyline();
  ~Skyline();

  void generate(unsigned, double, double);
  void step(); // one differential growth step
  bool getSettled() const { return settled; }
  void draw(UI &, bool);

  const std::vector<GrowthNode> &getNodes() const { return nodes; }
  const std::vector<Building> &getBuildings() const { return buildings; }

  static inline const int layers = 3;

private:
  std::vector<Vec2> outline(int, double, double, double, double);
  void addBuilding(const std::vector<Vec2> &, double, double, double, int);
  double growthMask(const Building &, const Vec2 &) const;
  Vec2 growthDir(const Building &, const Vec2 &) const;
  Vec2 outwardNormal(const Building &, int) const;
  double tipFactor(const Building &, int) const;
  void buildHash();
  int hashCell(const Vec2 &) const;
  void grow();
  double uniform(double, double);

  std::vector<GrowthNode> nodes;
  std::vector<Building> buildings;
  std::vector<Vec2> moves;

  // spatial hash (linked lists per cell) for the repulsion
  std::vector<int> head;
  std::vector<int> next;
  double hashMinX = 0, hashMinY = 0;
  int hashCols = 0, hashRows = 0;

  std::mt19937 rng;
  bool settled = false; // growth finished and at rest
  double halfWidth = 200, halfHeight = 133;

  // differential growth parameters (meters, per step)
  const double edge = 2.0;               // rest length of an edge
  const double splitLength = 3.0;        // longer edges are split
  const double repelRadius = 2.2 * edge; // distance kept between nodes
  const double kSpring = 0.3;
  const double kSmooth = 0.2;
  const double kRepel = 0.5;
  const double kPush = 0.1;              // outward push of growing tips
  const double growthRate = 0.03;        // insertions per edge and step
  const double tipAngle = 0.35;          // turning angle of a full tip [rad]
  const double maxStep = 0.3;
  const double maxGrowth = 2.5;          // node limit / initial nodes
  const double settleMove = 0.005;       // below this max move: settled
};

#endif
