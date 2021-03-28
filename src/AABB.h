#pragma once
#include <cstdint>
#include <algorithm>

#include "MathUtils.h"

namespace b2dl {

struct aabb_t {

  // make an aabb enclosing points a and b
  static aabb_t make(const Vec2 &a, const Vec2 &b) {
    return aabb_t{
      Vec2{std::min(a.x, b.x), std::min(a.y, b.y)},
      Vec2{std::max(a.x, b.x), std::max(a.y, b.y)}
    };
  }

  // return the area of the aabb
  float area() const {
    const float dx = max.x - min.x;
    const float dy = max.y - min.y;
    return dx * dy;
  }

  // return true if a contains b
  static bool contains(const aabb_t &a, const aabb_t &b) {
    if (b.min.x < a.min.x) return false;
    if (b.min.y < a.min.y) return false;
    if (b.max.x > a.max.x) return false;
    if (b.max.y > a.max.y) return false;
    return true;
  }

  // return true if two aabbs overlap
  static bool overlaps(const aabb_t &a, const aabb_t &b) {
    if (a.max.x < b.min.x) return false;
    if (a.min.x > b.max.x) return false;
    if (a.max.y < b.min.y) return false;
    if (a.min.y > b.max.y) return false;
    return true;
  }

  // find the union of two aabb
  static aabb_t find_union(const aabb_t &a, const aabb_t &b) {
    return aabb_t{
      std::min<float>(a.min.x, b.min.x),
      std::min<float>(a.min.y, b.min.y),
      std::max<float>(a.max.x, b.max.x),
      std::max<float>(a.max.y, b.max.y)
    };
  }

  // grow the aabb evenly by an amount
  static aabb_t grow(const aabb_t &a, float amount) {
    return aabb_t{
      a.min.x - amount,
      a.min.y - amount,
      a.max.x + amount,
      a.max.y + amount
    };
  }

  // evaluate if this aabb contains another
  bool contains(const aabb_t &a) const {
    return a.min.x >= min.x && a.min.y >= min.y && a.max.x <= max.x && a.max.y <= max.y;
  }

  // evaluate if point a is contained in this aabb
  bool contains(const Vec2 &a) const {
    return a.x >= min.x && a.x <= max.x && a.y >= min.y && a.y <= max.y;
  }

  // aabb minimum and maximum coordinates
  Vec2 min, max;
};

}  // b2dl
