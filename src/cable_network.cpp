#include "../include/cable_network.h"
#include <algorithm>
#include <cmath>

CableNetwork::CableNetwork() {}
CableNetwork::~CableNetwork() {}

double CableNetwork::uniform(double _a, double _b) {
  std::uniform_real_distribution<double> dist(_a, _b);
  return dist(rng);
}

int CableNetwork::uniformInt(int _a, int _b) {
  std::uniform_int_distribution<int> dist(_a, _b);
  return dist(rng);
}

// sag of a cable with length _L between two points _D apart (parabola approx.)
double CableNetwork::sagDepth(double _D, double _L) {
  return _L > _D ? std::sqrt(3 * _D * (_L - _D) / 8) : 0;
}

void CableNetwork::generate(unsigned _seed, double _halfWidth,
                            double _halfHeight,
                            const std::vector<Vec2> &_anchors) {
  particles.clear();
  links.clear();
  bends.clear();
  cables.clear();
  anchors.clear();
  rng.seed(_seed);
  halfWidth = _halfWidth;
  halfHeight = _halfHeight;
  time = 0;

  // anchors on the buildings (same order as given, fixed particles)
  for (const Vec2 &p : _anchors)
    anchors.push_back(addParticle(p, 0));
  anchorFrom = anchorTo = _anchors;
  if (anchors.empty())
    return;

  double density = 4 * halfWidth * halfHeight / referenceArea;
  auto scaled = [&](int _count) { return std::max(1, (int)std::round(_count * density)); };

  // free hanging vines
  for (int i = 0; i < scaled(hangingCount); ++i) {
    int a = randomAnchor();
    double maxLength = particles[a].pos.y + halfHeight - 3;
    if (maxLength < 4)
      continue;
    addCable(a, -1, uniform(4, std::min(maxLength, 22.0)), CableType::HANGING);
  }

  // cables draped between two anchors
  int draped = 0;
  for (int tries = 0; tries < 500 && draped < scaled(drapedCount); ++tries) {
    int a = randomAnchor(), b = randomAnchor();
    Vec2 d = particles[b].pos - particles[a].pos;
    if (std::abs(d.x) < 4 || std::abs(d.x) > 18 || std::abs(d.y) > 8)
      continue;
    double D = d.length();
    double L = D * uniform(1.05, 1.8);
    // keep the lowest point above the bottom of the screen
    double room = (particles[a].pos.y + particles[b].pos.y) / 2 + halfHeight - 2;
    while (sagDepth(D, L) > room && L > D * 1.02)
      L *= 0.95;
    addCable(a, b, L, CableType::DRAPED);
    draped++;
  }

  // tangles: vines with their lower end knotted into another cable
  int tangles = 0;
  for (int tries = 0; tries < 500 && tangles < scaled(tangleCount); ++tries) {
    const Cable &other = cables[uniformInt(0, cables.size() - 1)];
    if (other.ids.size() < 8)
      continue;
    int p = other.ids[uniformInt(3, other.ids.size() - 3)];
    int a = randomAnchor();
    Vec2 d = particles[p].pos - particles[a].pos;
    double D = d.length();
    // slightly taut and with some sideways offset, slack would curl into loops
    if (D < 3 || D > 10 || d.y > -1 || std::abs(d.x) < 1.5)
      continue;
    addCable(a, p, D * uniform(0.96, 1.0), CableType::TANGLED);
    tangles++;
  }

  settle(5.0);
}

int CableNetwork::addParticle(const Vec2 &_pos, double _invMass) {
  particles.push_back({_pos, _pos, {0, 0}, {0, 0}, _invMass});
  return particles.size() - 1;
}

// random anchor on screen (the buildings reach above it)
int CableNetwork::randomAnchor() {
  for (int tries = 0; tries < 50; ++tries) {
    int a = anchors[uniformInt(0, anchors.size() - 1)];
    Vec2 p = particles[a].pos;
    if (p.y < halfHeight - 0.5 && std::abs(p.x) < halfWidth - 0.5)
      return a;
  }
  return anchors[uniformInt(0, anchors.size() - 1)];
}

// in 2D the bending keeps a rope from unwinding a loop (it would have to
// pass a kink), so cables which cross themselves bend freely for a moment
void CableNetwork::unloop(double _dt) {
  for (Cable &c : cables)
    c.bendOff = std::max(0.0, c.bendOff - _dt);
  loopTimer -= _dt;
  if (loopTimer > 0)
    return;
  loopTimer = loopCheck;

  auto crosses = [&](Vec2 _a, Vec2 _b, Vec2 _c, Vec2 _d) {
    auto side = [](Vec2 _p, Vec2 _q, Vec2 _r) { return (_q - _p).cross(_r - _p); };
    return side(_a, _b, _c) * side(_a, _b, _d) < 0 &&
           side(_c, _d, _a) * side(_c, _d, _b) < 0;
  };
  for (Cable &c : cables) {
    int n = c.ids.size();
    bool looped = false;
    for (int i = 0; i + 1 < n && !looped; ++i)
      for (int j = i + 2; j + 1 < n && !looped; ++j)
        looped = crosses(particles[c.ids[i]].pos, particles[c.ids[i + 1]].pos,
                         particles[c.ids[j]].pos, particles[c.ids[j + 1]].pos);
    if (looped)
      c.bendOff = unbendTime;
  }
}

void CableNetwork::setAnchorTargets(const std::vector<Vec2> &_targets) {
  for (size_t k = 0; k < anchors.size() && k < _targets.size(); ++k) {
    anchorFrom[k] = particles[anchors[k]].pos;
    anchorTo[k] = _targets[k];
  }
}

void CableNetwork::moveAnchors(double _t) {
  for (size_t k = 0; k < anchors.size(); ++k) {
    Particle &p = particles[anchors[k]];
    p.pos = p.prev = lerp(anchorFrom[k], anchorTo[k], _t);
  }
}

// a cable between two anchors which move apart gets more rope instead of
// being torn (rest lengths grow with the distance of its ends)
void CableNetwork::payOut() {
  for (Cable &c : cables) {
    if (c.type == CableType::HANGING)
      continue; // both ends held: between anchors or tied into a cable
    int segments = c.ids.size() - 1;
    double chord = (particles[c.ids.back()].pos - particles[c.ids[0]].pos).length();
    if (chord < taut * c.segment * segments)
      continue;
    double slack = c.type == CableType::TANGLED ? tiedSlack : payOutSlack;
    c.segment = chord * slack / segments; // sag again
    for (int k = 0; k < segments; ++k)
      links[c.firstLink + k].rest = c.segment;
    for (int k = 0; k + 1 < segments; ++k)
      bends[c.firstBend + k].rest = 2 * c.segment;
  }
}

// cable from particle _start to particle _end (-1 for a free end)
void CableNetwork::addCable(int _start, int _end, double _length,
                            CableType _type) {
  int n = std::max(2, (int)std::round(_length / segmentLength));
  double rest = _length / n;
  double invMass = 1.0 / (linearDensity * rest);

  Vec2 a = particles[_start].pos;
  Vec2 b = _end >= 0 ? particles[_end].pos : a + Vec2(0, -_length);
  double sag = _end >= 0 ? sagDepth((b - a).length(), _length) : 0;

  Cable cable;
  cable.type = _type;
  cable.segment = rest;
  cable.shade = uniform(0, 1);
  cable.firstLink = links.size();
  cable.firstBend = bends.size();
  cable.ids.push_back(_start);
  // initial shape: parabola below the chord
  for (int i = 1; i < n; ++i) {
    double t = (double)i / n;
    Vec2 p = lerp(a, b, t) - Vec2(0, 4 * sag * t * (1 - t));
    cable.ids.push_back(addParticle(p, invMass));
  }
  cable.ids.push_back(_end >= 0 ? _end : addParticle(b, invMass));

  for (size_t i = 1; i < cable.ids.size(); ++i)
    links.push_back({cable.ids[i - 1], cable.ids[i], rest});
  for (size_t i = 2; i < cable.ids.size(); ++i)
    bends.push_back({cable.ids[i - 2], cable.ids[i], 2 * rest});
  cables.push_back(cable);
}

// let the cables come to rest before the simulation starts
// without bending: loops formed while falling into place can still open up
// (with bending a 2D rope can't unwind a loop without passing a kink)
void CableNetwork::settle(double _seconds) {
  bool wind = windOn;
  windOn = false;
  bendOn = false;
  int steps = _seconds * 60;
  for (int i = 0; i < steps; ++i) {
    for (int s = 0; s < 10; ++s)
      substep(1.0 / 600);
    // strong damping to kill the swinging
    for (auto &p : particles)
      p.vel *= 0.9;
  }
  windOn = wind;
  bendOn = true;
  time = 0;
}

void CableNetwork::clearForces() {
  for (auto &p : particles)
    p.force = {0, 0};
}

// gentle gusts travelling through the canopy
double CableNetwork::windAt(const Vec2 &_pos) const {
  return windStrength * (std::sin(0.4 * time + 0.15 * _pos.x) +
                         0.5 * std::sin(1.3 * time + 0.4 * _pos.y + 1.0));
}

// one position based dynamics substep (predict, project, update velocity)
void CableNetwork::substep(double _h) {
  time += _h;
  for (auto &p : particles) {
    if (p.invMass == 0)
      continue;
    Vec2 acc = gravity + p.force * p.invMass;
    if (windOn)
      acc.x += windAt(p.pos);
    p.vel += acc * _h;
    p.prev = p.pos;
    p.pos += p.vel * _h;
  }

  for (int it = 0; it < iterations; ++it) {
    // alternate the direction to avoid a bias along the cables
    int n = links.size();
    for (int k = 0; k < n; ++k)
      solveLink(links[it % 2 == 0 ? k : n - 1 - k], 1.0);
    // bending only resists being shortened (a vine can bend, but not kink)
    for (const Cable &c : cables) {
      if (!bendOn || c.bendOff > 0)
        continue;
      for (size_t k = 0; k + 2 < c.ids.size(); ++k) {
        const Link &l = bends[c.firstBend + k];
        if ((particles[l.b].pos - particles[l.a].pos).length() < l.rest)
          solveLink(l, bendStiffness);
      }
    }
  }

  double keep = std::max(0.0, 1 - damping * _h);
  for (auto &p : particles) {
    if (p.invMass == 0)
      continue;
    p.vel = (p.pos - p.prev) / _h * keep;
  }
}

// move both particles to restore the rest length (weighted by mass)
void CableNetwork::solveLink(const Link &_l, double _stiffness) {
  Particle &pa = particles[_l.a];
  Particle &pb = particles[_l.b];
  double w = pa.invMass + pb.invMass;
  if (w == 0)
    return;
  Vec2 d = pb.pos - pa.pos;
  double len = d.length();
  if (len < 1e-9)
    return;
  Vec2 corr = d * (_stiffness * (len - _l.rest) / (len * w));
  pa.pos += corr * pa.invMass;
  pb.pos -= corr * pb.invMass;
}

Vec2 CableNetwork::getPoint(const CablePoint &_cp) const {
  const Cable &c = cables[_cp.cable];
  int maxU = c.ids.size() - 1;
  double u = std::clamp(_cp.u, 0.0, (double)maxU);
  int k = std::min((int)u, maxU - 1);
  return lerp(particles[c.ids[k]].pos, particles[c.ids[k + 1]].pos, u - k);
}

Vec2 CableNetwork::getVelocity(const CablePoint &_cp) const {
  const Cable &c = cables[_cp.cable];
  int maxU = c.ids.size() - 1;
  double u = std::clamp(_cp.u, 0.0, (double)maxU);
  int k = std::min((int)u, maxU - 1);
  return lerp(particles[c.ids[k]].vel, particles[c.ids[k + 1]].vel, u - k);
}

// split a force onto the two particles of the segment
void CableNetwork::applyForce(const CablePoint &_cp, const Vec2 &_f) {
  const Cable &c = cables[_cp.cable];
  int maxU = c.ids.size() - 1;
  double u = std::clamp(_cp.u, 0.0, (double)maxU);
  int k = std::min((int)u, maxU - 1);
  double t = u - k;
  particles[c.ids[k]].force += _f * (1 - t);
  particles[c.ids[k + 1]].force += _f * t;
}

void CableNetwork::draw(UI &_ui, const CableStyle &_style) {
  // cables, between the translucent city and the solid pod
  for (const Cable &c : cables) {
    _ui.setAlpha(_style.alpha + _style.alphaSpread * c.shade);
    double width = c.type == CableType::DRAPED ? 2.5 : 2;
    for (size_t i = 1; i < c.ids.size(); ++i)
      _ui.drawThickLine(particles[c.ids[i - 1]].pos, particles[c.ids[i]].pos,
                        width);
    // mount on the building
    if (_style.mounts)
      _ui.fillCircle(particles[c.ids[0]].pos, 2.5 / _ui.scale);
  }
}
