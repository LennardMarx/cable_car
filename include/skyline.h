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
  double rest;      // rest length of its edges, fixed when created
  int born;         // growth step it was created in (hardens later)
  bool tip = false; // a branch grew out of it (dragged far away)
  Vec2 home;        // original position (building nodes)
};

// point on an edge of the original outline (moves with the deformation)
struct EdgePoint {
  int a, b; // node indices
  double t; // position between them
};

// window corner: between a left and a right wall point of its floor
struct WindowCorner {
  EdgePoint left, right;
  double u; // 0 at the left wall, 1 at the right wall
};

struct Window {
  WindowCorner corners[4]; // bottom left, bottom right, top right, top left
  bool lit;
};

// growing end of a branch, drags the outline along its heading
struct GrowthTip {
  int node;      // node index at the end of the branch
  Vec2 heading;  // growth direction
  double budget; // length still to grow [m]
  int depth;     // 0 main branch, 1 side branch, ...
  int cap;       // outline nodes on each side moving with the tip
  double width;  // half width of the branch at its start [m]
  double length; // total length to grow [m]
  int stuck = 0; // steps without progress
  int lastKink = 1; // direction of the last kink (+1 left, -1 right)
};

struct Building {
  std::vector<int> ring; // node indices along the outline, counter clockwise
  double cx;             // center line
  double base;           // ground level
  double height;         // original height
  // smooth warp of the upper tower: sway along the height, flaring out
  double swayAmp, swayFreq, swayPhase, flare;
  int maxNodes;          // safety limit for the growth
  std::vector<GrowthTip> tips;
  std::vector<Window> windows;
  int layer; // 0 far, 1 middle, 2 near
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

  // skyline meters -> world units (the game draws the city smaller)
  void setWorldScale(double _scale) { worldScale = _scale; }
  Vec2 nodePosition(int _node) const { return nodes[_node].pos * worldScale; }
  std::vector<int> pickAnchors(int, int);
  void draw(UI &, bool);

  const std::vector<GrowthNode> &getNodes() const { return nodes; }
  const std::vector<Building> &getBuildings() const { return buildings; }

  static inline const int layers = 3;
  static inline const int styles = 5;

  // corners of a futuristic tower (style, center, width, height, base)
  static std::vector<Vec2> outline(int, double, double, double, double);

private:
  void addBuilding(const std::vector<Vec2> &, double, double, double, int);
  double growthMask(const Building &, const Vec2 &) const;
  Vec2 growthDir(const Building &, const Vec2 &) const;
  Vec2 outwardNormal(const Building &, int) const;
  double growthBoost() const;
  double detailScale(const Building &, const Vec2 &) const;
  Vec2 morphOffset(const Building &, const Vec2 &) const;
  void seedTips(Building &);
  void moveTips(Building &, double);
  void updateTips(Building &);
  void branch(Building &, const GrowthTip &);
  bool capFree(const Building &, int, int) const;
  void addTip(Building &, int, const Vec2 &, double, int, int);
  void addWindows(Building &);
  std::vector<std::pair<EdgePoint, double>> crossings(const Building &,
                                                      double) const;
  Vec2 cornerPos(const WindowCorner &) const;
  void drawWindows(UI &, int);
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
  std::vector<Vec2> before; // node positions before the step

  std::mt19937 rng;
  double worldScale = 1;
  bool settled = false; // growth finished and at rest
  int growthSteps = 0;  // steps since the growth started
  double halfWidth = 200, halfHeight = 133;

  // differential growth parameters (meters, per step)
  const double edge = 1.6;               // rest length of an edge
  const double splitLength = 2.2 * edge; // longer edges are split (halves
                                         // stay above the rest length)
  const double repelRadius = 2.2 * edge; // distance kept between nodes
  const double kSpring = 0.3;
  const double kSmooth = 0.02; // low: rugged outlines
  const double kRepel = 0.5;
  const double jitter = 0.16;    // random roughness per step [m]
  const double roughFloor = 0.7; // share of the jitter after the burst
  const double taper = 0.3;      // finer detail at branch ends
  const double crownDepth = 70;  // m above the tower to full taper
  const double maxStep = 0.4;
  const double growthStart = -30; // world height where the growth begins
  const double growthRamp = 30;   // m above it to full growth
  // branches: fanning up from the towers into a flat canopy (Chasm City)
  const double tipSpeed = 0.06; // m per step at growth boost 1
  const double maxTipStep = 0.5;
  const double canopyHeight = 118; // top of the canopy (world height)
  const double canopyBand = 30;    // m below it where limbs turn sideways
  const double canopyTurn = 0.08;  // turn towards sideways per step in the band
  const double minSpread = 15, maxSpread = 40; // m along the canopy
  const int facadeTips = 2;                // branches per facade side
  const double roofFan = 0.8;     // angle spread of the roof branches [rad]
  const double spread = 1.8;      // sideways pull of the branches (width)
  const double branchLength = 15; // m grown per side branch (at the top,
                                  // more rarely further down)
  const double kinkLength = 20;   // m grown per kink (long straight limbs)
  const double minKink = 0.1, maxKink = 0.3; // kink angle [rad]
  const double tropism = 0;    // continuous bending (0: straight limbs)
  const double maxDeviation = 1.05; // max angle to the growth direction [rad]
  // thick branches: a cap of outline nodes moves with the tip, the stretched
  // flanks behind it fill with new nodes (the tree adds wood)
  const double limbShare = 0.22; // main branch width / tower width
  // morphing of the original building (smooth, the windows follow)
  const double minSway = 1, maxSway = 4; // m sideways at full growth
  const double minFlare = 0.25, maxFlare = 0.5; // widening towards the top
  const double morphTime = 2;     // s until the warp is mostly done
  const double kHome = 0.2;       // pull towards the warped position
  const double capStiffness = 0.2; // pull of the cap nodes into shape
  const double pointiness = 0.6;  // how far the cap sides trail the tip
  const double endWidth = 0.6;    // width at the end / width at the start
  const double woodRough = 0.15;  // share of the roughness on new wood
  const int maxDepth = 2;      // branching recursion
  const int maxTips = 14;      // growing tips per building
  // growth over time: fast burst, then easing off
  const double stepsPerSecond = 180; // nominal (3 steps at 60 fps)
  const double rampTime = 0.4;       // s to reach the burst
  const double burst = 4;            // rate multiplier at the start
  const double calm = 0.25;          // rate multiplier later on
  const double decayTime = 2.5;      // s of the burst decay
  const double maxGrowth = 3.5;      // node limit / initial nodes
  const double settleMove = 0.005;   // below this max move: settled
  const double hardenTime = 1.5;     // s until grown wood stops moving
  // windows (in the original building, they follow the deformation)
  const double floorPitch = 5; // m between window rows
  const double windowWidth = 2.2, windowHeight = 2.6;
  const double windowPitch = 4; // m between window columns
  const double wallMargin = 2;  // m from the walls
  const double litShare = 0.08; // share of lit windows
};

#endif
