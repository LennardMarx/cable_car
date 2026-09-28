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
  double rest; // rest length of its edges, fixed when created
  int born;    // growth step it was created in (hardens later)
};

// growing end of a branch, drags the outline along its heading
struct GrowthTip {
  int node;      // node index at the end of the branch
  Vec2 heading;  // growth direction
  double budget; // length still to grow [m]
  int depth;     // 0 main branch, 1 side branch, ...
  int stuck = 0; // steps without progress
};

struct Building {
  std::vector<int> ring; // node indices along the outline, counter clockwise
  double cx;             // center line
  double base;           // ground level
  double height;         // original height
  int maxNodes;          // safety limit for the growth
  std::vector<GrowthTip> tips;
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
  double growthBoost() const;
  double detailScale(const Building &, const Vec2 &) const;
  void seedTips(Building &);
  void moveTips(Building &, double);
  void updateTips(Building &);
  void branch(Building &, const GrowthTip &);
  void buildHash();
  int hashCell(const Vec2 &) const;
  void split();
  double uniform(double, double);

  std::vector<GrowthNode> nodes;
  std::vector<Building> buildings;
  std::vector<Vec2> moves;

  // spatial hash (linked lists per cell) for the repulsion
  std::vector<int> head;
  std::vector<int> next;
  double hashMinX = 0, hashMinY = 0;
  int hashCols = 0, hashRows = 0;

  std::vector<int> ringPos; // position of every node in its ring
  std::vector<Vec2> before;  // node positions before the step

  std::mt19937 rng;
  bool settled = false; // growth finished and at rest
  int growthSteps = 0;  // steps since the growth started
  double halfWidth = 200, halfHeight = 133;

  // differential growth parameters (meters, per step)
  const double edge = 1.6;               // rest length of an edge
  const double splitLength = 2.2 * edge; // longer edges are split (halves
                                         // stay above the rest length)
  const double repelRadius = 2.2 * edge; // distance kept between nodes
  const double kSpring = 0.3;
  const double kSmooth = 0.02;  // low: rugged outlines
  const double kRepel = 0.5;
  const double jitter = 0.16;   // random roughness per step [m]
  const double taper = 0.6;     // finer detail at branch ends
  const double crownDepth = 70; // m above the tower to full taper
  const double maxStep = 0.4;
  const double growthStart = 0; // world height where the growth begins
  const double growthRamp = 30; // m above it to full growth
  // branches (dead trees: long limbs, few side branches, sharp kinks)
  const double tipSpeed = 0.06; // m per step at growth boost 1
  const double maxTipStep = 0.5;
  const double minLimb = 35, maxLimb = 70;   // length of main branches [m]
  const int facadeTips = 3;                  // branches per facade side
  const double branchLength = 15;            // m grown per side branch
  const double kinkLength = 6;               // m grown per kink
  const double minKink = 0.2, maxKink = 0.6; // kink angle [rad]
  const double tropism = 0.01; // bending towards the growth direction
  const int maxDepth = 2;      // branching recursion
  const int maxTips = 10;      // growing tips per building
  // growth over time: fast burst, then easing off
  const double stepsPerSecond = 180; // nominal (3 steps at 60 fps)
  const double rampTime = 0.4;       // s to reach the burst
  const double burst = 4;            // rate multiplier at the start
  const double calm = 0.25;          // rate multiplier later on
  const double decayTime = 2.5;      // s of the burst decay
  const double maxGrowth = 3.5;      // node limit / initial nodes
  const double settleMove = 0.005;   // below this max move: settled
  const double hardenTime = 1.5;     // s until grown wood stops moving
};

#endif
