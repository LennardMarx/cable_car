#ifndef VEC2_H
#define VEC2_H

#pragma once
#include <cmath>

// small 2D vector helper (world units are meters, y pointing up)
struct Vec2 {
  double x = 0;
  double y = 0;

  Vec2() = default;
  Vec2(double _x, double _y) : x(_x), y(_y) {}

  Vec2 operator+(const Vec2 &_v) const { return {x + _v.x, y + _v.y}; }
  Vec2 operator-(const Vec2 &_v) const { return {x - _v.x, y - _v.y}; }
  Vec2 operator-() const { return {-x, -y}; }
  Vec2 operator*(double _s) const { return {x * _s, y * _s}; }
  Vec2 operator/(double _s) const { return {x / _s, y / _s}; }
  Vec2 &operator+=(const Vec2 &_v) {
    x += _v.x;
    y += _v.y;
    return *this;
  }
  Vec2 &operator-=(const Vec2 &_v) {
    x -= _v.x;
    y -= _v.y;
    return *this;
  }
  Vec2 &operator*=(double _s) {
    x *= _s;
    y *= _s;
    return *this;
  }

  double dot(const Vec2 &_v) const { return x * _v.x + y * _v.y; }
  // z-component of the 3D cross product (torque of a force _v at lever *this)
  double cross(const Vec2 &_v) const { return x * _v.y - y * _v.x; }
  double length() const { return std::sqrt(x * x + y * y); }

  Vec2 normalized() const {
    double l = length();
    return l > 1e-12 ? Vec2(x / l, y / l) : Vec2(0, 0);
  }
  // rotated by +90 degrees
  Vec2 perp() const { return {-y, x}; }
  Vec2 rotated(double _ang) const {
    double c = std::cos(_ang), s = std::sin(_ang);
    return {c * x - s * y, s * x + c * y};
  }
  // shortened to _max if longer
  Vec2 clampedLength(double _max) const {
    double l = length();
    return l > _max ? *this * (_max / l) : *this;
  }
};

inline Vec2 operator*(double _s, const Vec2 &_v) { return _v * _s; }
inline Vec2 lerp(const Vec2 &_a, const Vec2 &_b, double _t) {
  return _a + (_b - _a) * _t;
}

#endif
