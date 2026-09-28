#ifndef ARM_H
#define ARM_H

#pragma once
#include "cable_network.h"
#include "vec2.h"

enum class ArmState {
  RETRACTED, // folded in, looking for a grip
  REACHING,  // hook travelling to a point on a cable
  GRASPING   // hook closed around a cable, arm carries load
};

// telescopic two-link arm with a hook at its end
class Arm {
public:
  Arm(const Vec2 &, const Vec2 &);
  ~Arm();

  void solveIK(const Vec2 &, const Vec2 &);

  Vec2 mount;   // shoulder position in pod frame
  Vec2 natural; // preferred reaching direction in pod frame

  ArmState state = ArmState::RETRACTED;
  CablePoint grip;      // grasped or targeted point on a cable
  Vec2 hook;            // desired hook position (world)
  Vec2 elbow;           // elbow position from IK (world)
  Vec2 tip;             // actual end of the arm from IK (world)
  double extension = 1; // telescope factor of both links
  double cooldown = 0;  // time until the arm may change its grip again
  Vec2 force;           // force the arm applies on the pod

  static inline const double linkBase = 1.4; // link length at extension 1
  static inline const double minExtension = 0.6;
  static inline const double maxExtension = 1.8;
  static inline const double maxReach = 2 * linkBase * maxExtension;

private:
  int elbowSide = 1;
};

#endif
