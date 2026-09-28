#ifndef CABLE_CAR_H
#define CABLE_CAR_H

#pragma once
#include "UI.h"
#include "arm.h"
#include "cable_network.h"
#include "vec2.h"
#include <array>
#include <limits>

// pod with four telescopic arms climbing through the vines
class CableCar {
public:
  CableCar();
  ~CableCar();

  void reset(const CableNetwork &, const Vec2 &);
  void plan(CableNetwork &, double);  // gait and reference, once per frame
  void step(CableNetwork &, double);  // forces and integration, every substep
  void updateKinematics(const CableNetwork &);
  void draw(UI &, bool);

  void setTarget(const Vec2 &_target) { target = _target; }
  Vec2 getPosition() const { return pos; }
  double getAngle() const { return theta; }
  void toggleController() { controllerState = !controllerState; }
  bool getControllerState() const { return controllerState; }
  int countArms(ArmState) const;

private:
  Vec2 toWorld(const Vec2 &_local) const { return pos + _local.rotated(theta); }
  Vec2 shoulder(int) const;
  Vec2 naturalDir(int) const;
  Vec2 idealGrip(int) const;
  double gripCost(int, const CablePoint &, const CableNetwork &, bool) const;
  CablePoint findGrip(int, const CableNetwork &) const;
  int grabAll(const CableNetwork &);
  void updateReference(double);
  void climb(int, const CableNetwork &, double);
  void swapGrip(const CableNetwork &);
  void moveHook(Arm &, const Vec2 &, double, const Vec2 &);
  void distributeForces(double);

  std::array<Arm, 4> arms;

  // pod states
  Vec2 pos;
  Vec2 vel;
  double theta = 0; // orientation
  double omega = 0; // angular velocity

  // reference trajectory towards the target
  Vec2 ref;
  Vec2 refVel;
  Vec2 target;
  bool controllerState = true;

  static inline const double inf = std::numeric_limits<double>::infinity();
  const Vec2 gravity{0, -9.81};

  // pod
  const double mass = 40;
  const double halfWidth = 1.2;
  const double halfHeight = 0.8;
  const double inertia =
      mass * (4 * halfWidth * halfWidth + 4 * halfHeight * halfHeight) / 12;
  const double linearDrag = 4;  // N/(m/s)
  const double angularDrag = 5; // Nm/(rad/s)

  // controller (per unit mass / inertia)
  const double kp = 9, kd = 6;               // position
  const double kTheta = 120, kOmega = 22;    // orientation
  const double maxSpeed = 5.0;               // reference speed [m/s]
  const double maxAccel = 5.0;               // reference acceleration
  const double maxLag = 2.0;                 // max distance reference-pod
  const double maxArmForce = 1.3 * mass * 9.81;
  const double sideCompliance = 0.3;         // cost of forces across an arm

  // arms
  const double grabReach = 0.85 * Arm::maxReach;
  const double softReach = 0.9 * Arm::maxReach; // mechanical stop starts
  const double lostReach = 1.05 * Arm::maxReach;
  const double minReach = 1.0;
  const double comfortReach = 3.0;
  const double restReach = 1.3;
  const double lead = 3.0;           // how far the grips reach ahead [m]
  const double stopStiffness = 80 * mass;
  const double stopDamping = 15 * mass;
  const double hookSpeed = 15;       // m/s
  const double climbSpeed = 2.5;     // m/s
  const double regripTime = 0.4;     // s before a new grip may be released
  const double swapThreshold = 1.5;  // m improvement to change a grip
  const double crowding = 1.2;       // min distance between two hooks [m]
};

#endif
