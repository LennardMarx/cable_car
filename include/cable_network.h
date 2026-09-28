#ifndef CABLE_NETWORK_H
#define CABLE_NETWORK_H

#pragma once
#include "UI.h"
#include "vec2.h"
#include <random>
#include <vector>

struct Particle {
  Vec2 pos;
  Vec2 prev;        // position before the current substep
  Vec2 vel;
  Vec2 force;       // external force (hooks) for the current substep
  double invMass;   // 0 -> fixed anchor
};

// distance constraint between two particles
struct Link {
  int a;
  int b;
  double rest;
};

enum class CableType {
  HANGING, // one end on the canopy, one end free
  DRAPED,  // both ends on the canopy
  TANGLED  // one end on the canopy, one end knotted into another cable
};

struct Cable {
  std::vector<int> ids; // particle indices from the first to the last end
  CableType type;
  double segment; // rest length between particles
  double shade;   // [0, 1] for drawing variation
  int firstLink;  // its links and bends are stored consecutively
  int firstBend;
};

// point along a cable: u = segment index + fraction inside the segment
struct CablePoint {
  int cable = -1;
  double u = 0;
  bool valid() const { return cable >= 0; }
};

// vines hanging from the buildings, simulated with position based dynamics
// the anchors are kinematic and follow the growing city
class CableNetwork {
public:
  CableNetwork();
  ~CableNetwork();

  void generate(unsigned, double, double, const std::vector<Vec2> &);
  void clearForces();
  void substep(double);

  // anchors move from their current to new positions over the substeps
  void setAnchorTargets(const std::vector<Vec2> &);
  void moveAnchors(double); // 0 ... 1 of the way
  void payOut();            // lengthen cables pulled taut by their anchors

  Vec2 getPoint(const CablePoint &) const;
  Vec2 getVelocity(const CablePoint &) const;
  void applyForce(const CablePoint &, const Vec2 &);

  const std::vector<Particle> &getParticles() const { return particles; }
  const std::vector<Cable> &getCables() const { return cables; }

  void toggleWind() { windOn = !windOn; }
  bool getWindOn() const { return windOn; }

  void draw(UI &);

  static inline const double segmentLength = 0.5; // m
  static inline const double linearDensity = 2.0; // kg/m

private:
  int addParticle(const Vec2 &, double);
  void addCable(int, int, double, CableType);
  void settle(double);
  void solveLink(const Link &, double);
  int randomAnchor();
  double windAt(const Vec2 &) const;
  double uniform(double, double);
  int uniformInt(int, int);
  static double sagDepth(double, double);

  std::vector<Particle> particles;
  std::vector<Link> links;
  std::vector<Link> bends; // soft links over two segments (bending)
  std::vector<Cable> cables;
  std::vector<int> anchors; // particles on the buildings cables hang from
  std::vector<Vec2> anchorFrom, anchorTo;

  std::mt19937 rng;
  double halfWidth = 30, halfHeight = 20; // visible world extents
  double time = 0;
  bool windOn = true;
  bool bendOn = true;

  const Vec2 gravity{0, -9.81};
  const double damping = 0.05;       // velocity damping [1/s]
  const int iterations = 2;          // constraint iterations per substep
  const double bendStiffness = 0.05; // [0, 1] per iteration
  const double windStrength = 0.6;   // m/s^2
  const double taut = 0.97;          // chord / length that pays out rope
};

#endif
