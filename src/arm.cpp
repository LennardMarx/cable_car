#include "../include/arm.h"
#include <algorithm>
#include <cmath>

Arm::Arm(const Vec2 &_mount, const Vec2 &_natural)
    : mount(_mount), natural(_natural.normalized()) {}
Arm::~Arm() {}

// inverse kinematics from the shoulder to the hook
// both links telescope equally, the elbow bends away from the pod center
void Arm::solveIK(const Vec2 &_shoulder, const Vec2 &_center) {
  Vec2 d = hook - _shoulder;
  double dist = d.length();
  Vec2 dir = dist > 1e-6 ? d / dist : Vec2(0, 1);

  // telescope so the elbow keeps a comfortable bend
  extension = std::clamp(dist / (2 * linkBase * 0.8), minExtension, maxExtension);
  double l = linkBase * extension;
  double reach = std::min(dist, 2 * l);

  // law of cosines: angle between shoulder-hook line and the first link
  double alpha = std::acos(std::clamp(reach / (2 * l), -1.0, 1.0));
  Vec2 e1 = _shoulder + dir.rotated(alpha) * l;
  Vec2 e2 = _shoulder + dir.rotated(-alpha) * l;

  // pick the elbow further away from the pod (hysteresis against flipping)
  double d1 = (e1 - _center).length();
  double d2 = (e2 - _center).length();
  if (elbowSide == 1 && d2 > d1 + 0.3)
    elbowSide = -1;
  else if (elbowSide == -1 && d1 > d2 + 0.3)
    elbowSide = 1;
  elbow = elbowSide == 1 ? e1 : e2;

  // equals the hook when it is in reach
  tip = elbow + (hook - elbow).normalized() * l;
}
