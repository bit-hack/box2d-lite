/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#pragma once

#include "MathUtils.h"
#include "AABB.h"
#include "BVH.h"

namespace b2dl {

struct Body
{
  Body();

  void Set(const Vec2& width, float mass);

  void AddForce(const Vec2& f);

  void GetAABB(aabb_t &out) const;

  bool IsStatic() const {
    return invMass == 0.f;
  }

  Vec2 position;
  float rotation;

  Vec2 velocity;
  float angularVelocity;

  Vec2 force;
  float torque;

  Vec2 width;

  float friction;
  float mass, invMass;
  float I, invI;

  bvh_index_t bvh_index;
};

}  // namespace b2dl
