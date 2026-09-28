#include "../include/cable_car.h"
#include <algorithm>
#include <cmath>
#include <vector>

// arms at the four corners: top left, top right, bottom right, bottom left
// upper arms reach up and outwards, lower arms mostly sideways
CableCar::CableCar()
    : arms{Arm({-1.0, 0.6}, {-0.6, 0.8}), Arm({1.0, 0.6}, {0.6, 0.8}),
           Arm({1.0, -0.6}, {0.9, 0.45}), Arm({-1.0, -0.6}, {-0.9, 0.45})} {}
CableCar::~CableCar() {}

Vec2 CableCar::shoulder(int _i) const { return toWorld(arms[_i].mount); }

Vec2 CableCar::naturalDir(int _i) const {
  return arms[_i].natural.rotated(theta);
}

int CableCar::countArms(ArmState _state) const {
  int count = 0;
  for (const Arm &arm : arms)
    count += arm.state == _state;
  return count;
}

// where the arm would like to hold on: out in its natural direction and
// shifted towards the target, so the grips move ahead of the pod
Vec2 CableCar::idealGrip(int _i) const {
  Vec2 ahead = (target - pos).clampedLength(lead);
  return shoulder(_i) + naturalDir(_i) * comfortReach + ahead;
}

// how bad a grip at a cable point is for arm _i (inf if not reachable)
// _current: the arm already holds this point (strain instead of rejection)
double CableCar::gripCost(int _i, const CablePoint &_grip,
                          const CableNetwork &_net, bool _current) const {
  Vec2 p = _net.getPoint(_grip);
  Vec2 s = shoulder(_i);
  double dist = (p - s).length();
  if (!_current && (dist > grabReach || dist < minReach))
    return inf;

  double cost = (p - idealGrip(_i)).length();
  if (dist > grabReach) // strained arm
    cost += 20 * (dist - grabReach);
  if (dist < minReach) // cramped arm
    cost += 5 * (minReach - dist);

  // reaching across the pod
  if ((p - s).dot(naturalDir(_i)) < -0.2 * dist)
    cost += 3;

  // floppy tip of a free hanging vine
  const Cable &cable = _net.getCables()[_grip.cable];
  if (cable.type == CableType::HANGING &&
      _grip.u > cable.ids.size() - 1 - 1.5 / cable.segment)
    cost += 1.5;

  // keep the hooks apart
  for (int j = 0; j < 4; ++j) {
    const Arm &other = arms[j];
    if (j == _i || other.state == ArmState::RETRACTED)
      continue;
    bool sameCable = other.grip.cable == _grip.cable &&
                     std::abs(other.grip.u - _grip.u) * cable.segment < crowding;
    if (sameCable || (_net.getPoint(other.grip) - p).length() < crowding)
      cost += 4;
  }
  return cost;
}

// best reachable point on all cables for arm _i
CablePoint CableCar::findGrip(int _i, const CableNetwork &_net) const {
  CablePoint best;
  double bestCost = inf;
  Vec2 s = shoulder(_i);
  const auto &cables = _net.getCables();
  const auto &particles = _net.getParticles();

  for (int c = 0; c < (int)cables.size(); ++c) {
    int maxU = cables[c].ids.size() - 1;
    for (int k = 0; k <= maxU; ++k) {
      // quick rejection of far away segments
      if ((particles[cables[c].ids[k]].pos - s).length() > grabReach + 1)
        continue;
      for (double f = 0; f < 1 && k + f <= maxU; f += 0.5) {
        CablePoint cp{c, k + f};
        double cost = gripCost(_i, cp, _net, false);
        if (cost < bestCost) {
          bestCost = cost;
          best = cp;
        }
      }
    }
  }
  return best;
}

// grab the best points directly (used when spawning)
int CableCar::grabAll(const CableNetwork &_net) {
  for (int i = 0; i < 4; ++i) {
    arms[i].state = ArmState::RETRACTED;
    arms[i].grip = CablePoint();
    arms[i].cooldown = 0;
    arms[i].hook = shoulder(i) + naturalDir(i) * restReach;
  }
  int count = 0;
  for (int i = 0; i < 4; ++i) {
    CablePoint grip = findGrip(i, _net);
    if (!grip.valid())
      continue;
    arms[i].state = ArmState::GRASPING;
    arms[i].grip = grip;
    arms[i].hook = _net.getPoint(grip);
    count++;
  }
  return count;
}

// spawn the pod where it can hold on with as many arms as possible
void CableCar::reset(const CableNetwork &_net, const Vec2 &_hint) {
  theta = 0;
  omega = 0;
  vel = {0, 0};

  Vec2 bestPos = _hint;
  int bestCount = -1;
  double bestDist = inf;
  for (double x = -24; x <= 24; x += 2) {
    for (double y = -14; y <= 12; y += 2) {
      pos = target = {x, y};
      int count = grabAll(_net);
      double dist = (pos - _hint).length();
      if (count > bestCount || (count == bestCount && dist < bestDist)) {
        bestCount = count;
        bestDist = dist;
        bestPos = pos;
      }
    }
  }
  pos = ref = target = bestPos;
  refVel = {0, 0};
  grabAll(_net);
  updateKinematics(_net);
}

// smooth reference moving towards the target with limited speed and accel.
void CableCar::updateReference(double _dt) {
  if (!controllerState || countArms(ArmState::GRASPING) == 0) {
    ref = pos;
    refVel = vel;
    return;
  }
  Vec2 toTarget = target - ref;
  double dist = toTarget.length();
  // capped speed that still allows braking in time
  double speed = std::min({maxSpeed, std::sqrt(2 * maxAccel * dist), 2 * dist});
  Vec2 desiredVel = dist > 1e-6 ? toTarget / dist * speed : Vec2(0, 0);
  refVel += (desiredVel - refVel).clampedLength(maxAccel * _dt);
  ref += refVel * _dt;

  // don't let the reference run away from a stuck pod
  Vec2 lag = ref - pos;
  if (lag.length() > maxLag) {
    ref = pos + lag.normalized() * maxLag;
    refVel *= 0.9;
  }
}

// move a free hook towards a goal, but never beyond the arm's reach
void CableCar::moveHook(Arm &_arm, const Vec2 &_goal, double _maxStep,
                        const Vec2 &_shoulder) {
  _arm.hook += (_goal - _arm.hook).clampedLength(_maxStep);
  Vec2 d = _arm.hook - _shoulder;
  if (d.length() > Arm::maxReach)
    _arm.hook = _shoulder + d.normalized() * Arm::maxReach;
}

// slide a grasping hook along its cable towards a better position
void CableCar::climb(int _i, const CableNetwork &_net, double _dt) {
  Arm &arm = arms[_i];
  const Cable &cable = _net.getCables()[arm.grip.cable];
  double maxU = cable.ids.size() - 1;
  double du = climbSpeed * _dt / cable.segment;
  double probe = 0.25;

  CablePoint up = arm.grip, down = arm.grip;
  up.u = std::min(maxU, arm.grip.u + probe);
  down.u = std::max(0.0, arm.grip.u - probe);
  double c0 = gripCost(_i, arm.grip, _net, true);
  double cUp = gripCost(_i, up, _net, true);
  double cDown = gripCost(_i, down, _net, true);

  if (cUp < c0 - 0.02 && cUp <= cDown)
    arm.grip.u = std::min(maxU, arm.grip.u + du);
  else if (cDown < c0 - 0.02)
    arm.grip.u = std::max(0.0, arm.grip.u - du);
}

// gait: release the grip which can be improved the most and reach for the
// better one, as long as enough other arms keep holding on
void CableCar::swapGrip(const CableNetwork &_net) {
  if (countArms(ArmState::REACHING) > 0 || countArms(ArmState::GRASPING) < 3)
    return;
  int best = -1;
  double bestGain = swapThreshold;
  CablePoint bestGrip;
  for (int i = 0; i < 4; ++i) {
    if (arms[i].state != ArmState::GRASPING || arms[i].cooldown > 0)
      continue;
    CablePoint grip = findGrip(i, _net);
    if (!grip.valid())
      continue;
    double gain = gripCost(i, arms[i].grip, _net, true) -
                  gripCost(i, grip, _net, false);
    if (gain > bestGain) {
      bestGain = gain;
      best = i;
      bestGrip = grip;
    }
  }
  if (best >= 0) {
    arms[best].state = ArmState::REACHING;
    arms[best].grip = bestGrip;
  }
}

void CableCar::plan(CableNetwork &_net, double _dt) {
  updateReference(_dt);

  for (int i = 0; i < 4; ++i) {
    Arm &arm = arms[i];
    Vec2 s = shoulder(i);
    arm.cooldown = std::max(0.0, arm.cooldown - _dt);

    if (arm.state == ArmState::GRASPING) {
      // the grip slips when the cable drags the hook out of reach
      if ((_net.getPoint(arm.grip) - s).length() > lostReach) {
        arm.state = ArmState::RETRACTED;
        arm.cooldown = 0.3;
        continue;
      }
      if (controllerState)
        climb(i, _net, _dt);
    } else if (arm.state == ArmState::REACHING) {
      Vec2 goal = _net.getPoint(arm.grip);
      // target swung out of reach -> choose again
      if ((goal - s).length() > softReach) {
        arm.grip = findGrip(i, _net);
        if (!arm.grip.valid()) {
          arm.state = ArmState::RETRACTED;
          continue;
        }
        goal = _net.getPoint(arm.grip);
      }
      moveHook(arm, goal, hookSpeed * _dt, s);
      if ((arm.hook - goal).length() < 0.1) {
        arm.state = ArmState::GRASPING;
        arm.cooldown = regripTime;
      }
    } else {
      if (controllerState && arm.cooldown == 0) {
        CablePoint grip = findGrip(i, _net);
        if (grip.valid()) {
          arm.grip = grip;
          arm.state = ArmState::REACHING;
        }
      }
      moveHook(arm, s + naturalDir(i) * restReach, hookSpeed * 0.5 * _dt, s);
    }
  }

  if (controllerState)
    swapGrip(_net);
}

// solve [3x3] * y = b with Cramer's rule
static bool solve3(const double _m[3][3], const double _b[3], double _y[3]) {
  auto det = [](const double m[3][3]) {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  };
  double d = det(_m);
  if (std::abs(d) < 1e-12)
    return false;
  for (int c = 0; c < 3; ++c) {
    double m[3][3];
    for (int r = 0; r < 3; ++r)
      for (int k = 0; k < 3; ++k)
        m[r][k] = k == c ? _b[r] : _m[r][k];
    _y[c] = det(m) / d;
  }
  return true;
}

// controller: desired force and torque on the pod, distributed onto the
// grasping arms (weighted least norm, forces along the arms are cheaper)
void CableCar::distributeForces() {
  std::vector<int> active;
  for (int i = 0; i < 4; ++i) {
    arms[i].force = {0, 0};
    if (arms[i].state == ArmState::GRASPING)
      active.push_back(i);
  }
  if (!controllerState || active.empty())
    return;

  // PD on the reference + gravity and drag compensation
  Vec2 force = (ref - pos) * (mass * kp) + (refVel - vel) * (mass * kd) -
               gravity * mass + vel * linearDrag;
  double angle = std::remainder(theta, 2 * M_PI);
  double torque = inertia * (-kTheta * angle - kOmega * omega);
  double b[3] = {force.x, force.y, torque};

  // wrench of arm i: A_i * F_i with A_i = [1 0; 0 1; -r.y r.x]
  // weights W_i^-1 = u u^T + side * n n^T (u: along the arm)
  struct Block {
    Vec2 r;
    double w11, w12, w22;
  };
  std::vector<Block> blocks;
  double m[3][3] = {{1e-3, 0, 0}, {0, 1e-3, 0}, {0, 0, 1e-3}};
  for (int i : active) {
    Vec2 s = shoulder(i);
    Vec2 u = (arms[i].hook - s).normalized();
    if (u.length() == 0)
      u = {0, 1};
    Vec2 n = u.perp();
    Block bl{s - pos, u.x * u.x + sideCompliance * n.x * n.x,
             u.x * u.y + sideCompliance * n.x * n.y,
             u.y * u.y + sideCompliance * n.y * n.y};
    blocks.push_back(bl);

    // A W A^T
    double t1 = -bl.r.y * bl.w11 + bl.r.x * bl.w12;
    double t2 = -bl.r.y * bl.w12 + bl.r.x * bl.w22;
    m[0][0] += bl.w11;
    m[0][1] += bl.w12;
    m[1][1] += bl.w22;
    m[0][2] += t1;
    m[1][2] += t2;
    m[2][2] += -bl.r.y * t1 + bl.r.x * t2;
  }
  m[1][0] = m[0][1];
  m[2][0] = m[0][2];
  m[2][1] = m[1][2];

  double y[3];
  if (!solve3(m, b, y))
    return;

  // F_i = W_i A_i^T y, limited by the motor strength
  for (size_t k = 0; k < active.size(); ++k) {
    const Block &bl = blocks[k];
    double ax = y[0] - bl.r.y * y[2];
    double ay = y[1] + bl.r.x * y[2];
    Vec2 f(bl.w11 * ax + bl.w12 * ay, bl.w12 * ax + bl.w22 * ay);
    arms[active[k]].force = f.clampedLength(maxArmForce);
  }
}

// one substep of the pod dynamics, arm forces react on the cables
void CableCar::step(CableNetwork &_net, double _h) {
  // grasping hooks move with their cables
  for (Arm &arm : arms)
    if (arm.state == ArmState::GRASPING)
      arm.hook = _net.getPoint(arm.grip);

  distributeForces();

  Vec2 totalForce = gravity * mass - vel * linearDrag;
  double totalTorque = -omega * angularDrag;

  for (int i = 0; i < 4; ++i) {
    Arm &arm = arms[i];
    if (arm.state != ArmState::GRASPING)
      continue;
    Vec2 s = shoulder(i);
    Vec2 r = s - pos;
    Vec2 d = arm.hook - s;
    double dist = d.length();

    // mechanical stop: the arm cannot stretch beyond its reach
    if (dist > softReach) {
      Vec2 u = d / dist;
      Vec2 relVel = _net.getVelocity(arm.grip) - (vel + r.perp() * omega);
      double stop =
          stopStiffness * (dist - softReach) + stopDamping * relVel.dot(u);
      if (stop > 0)
        arm.force += u * stop;
    }

    totalForce += arm.force;
    totalTorque += r.cross(arm.force);
    _net.applyForce(arm.grip, -arm.force); // actio = reactio
  }

  // semi-implicit euler
  vel += totalForce / mass * _h;
  pos += vel * _h;
  omega += totalTorque / inertia * _h;
  theta += omega * _h;
}

void CableCar::updateKinematics(const CableNetwork &_net) {
  for (int i = 0; i < 4; ++i) {
    if (arms[i].state == ArmState::GRASPING)
      arms[i].hook = _net.getPoint(arms[i].grip);
    arms[i].solveIK(shoulder(i), pos);
  }
}

void CableCar::draw(UI &_ui, bool _debug) {
  const double sleeve = Arm::linkBase * Arm::minExtension;

  // arms: thick outer sleeve and thinner telescoping rod per link
  for (int i = 0; i < 4; ++i) {
    const Arm &arm = arms[i];
    Vec2 s = shoulder(i);
    Vec2 d1 = (arm.elbow - s).normalized();
    Vec2 d2 = (arm.tip - arm.elbow).normalized();

    _ui.setAlpha(255);
    _ui.drawThickLine(s, arm.elbow, 3);
    _ui.drawThickLine(arm.elbow, arm.tip, 3);
    _ui.drawThickLine(s, s + d1 * sleeve, 7);
    _ui.drawThickLine(arm.elbow, arm.elbow + d2 * sleeve, 6);
    _ui.fillCircle(arm.elbow, 5 / _ui.scale);

    // hook: open claw while travelling, closed when holding a cable
    Vec2 n = d2.perp();
    double r = 0.28;
    if (arm.state == ArmState::GRASPING) {
      _ui.fillCircle(arm.tip, 4 / _ui.scale);
      _ui.drawThickLine(arm.tip, arm.tip + d2 * r + n * r * 0.6, 2.5);
      _ui.drawThickLine(arm.tip, arm.tip + d2 * r - n * r * 0.6, 2.5);
    } else {
      _ui.drawThickLine(arm.tip, arm.tip + d2 * r + n * r, 2.5);
      _ui.drawThickLine(arm.tip + d2 * r + n * r, arm.tip + d2 * r * 1.6, 2.5);
      _ui.drawThickLine(arm.tip, arm.tip - n * r * 0.5, 2.5);
    }
  }

  // pod: chamfered box with a window band
  double w = halfWidth, h = halfHeight, c = 0.35;
  std::vector<Vec2> body = {{-w + c, h},  {w - c, h},  {w, h - c},
                            {w, -h + c},  {w - c, -h}, {-w + c, -h},
                            {-w, -h + c}, {-w, h - c}};
  for (Vec2 &p : body)
    p = toWorld(p);
  _ui.setAlpha(255);
  _ui.fillPolygon(body);

  std::vector<Vec2> window = {{-0.85, 0.35}, {0.85, 0.35}, {0.85, -0.1},
                              {-0.85, -0.1}};
  for (Vec2 &p : window)
    p = toWorld(p);
  _ui.setDrawColor(69, 133, 136, 255); // gruv-blue
  _ui.fillPolygon(window);
  for (int i = 0; i < 4; ++i)
    _ui.fillCircle(shoulder(i), 3 / _ui.scale);

  if (!_debug)
    return;
  // debug: reach, ideal grips, arm forces, reference
  for (int i = 0; i < 4; ++i) {
    Vec2 s = shoulder(i);
    _ui.setAlpha(40);
    _ui.drawCircle(s, grabReach);
    _ui.setAlpha(160);
    Vec2 ideal = idealGrip(i);
    _ui.drawLine(ideal - Vec2(0.2, 0.2), ideal + Vec2(0.2, 0.2));
    _ui.drawLine(ideal - Vec2(0.2, -0.2), ideal + Vec2(0.2, -0.2));
    _ui.setAlpha(220);
    _ui.drawThickLine(s, s + arms[i].force * 0.004, 2);
  }
  _ui.setAlpha(160);
  _ui.drawCircle(ref, 0.25);
}
