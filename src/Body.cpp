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

#include <algorithm>

#include "Body.h"

using namespace b2dl;

Body::Body()
  : position{ 0.f, 0.f }
  , rotation(0.f)
  , velocity{0.f, 0.f}
  , angularVelocity(0.f)
  , force{ 0.f, 0.f }
  , torque(0.f)
  , width{1.f, 1.f}
  , friction(0.2f)
  , mass(FLT_MAX)
  , invMass(0.f)
  , I(FLT_MAX)
  , invI(0.f)
  , bvh_index(bvh_invalid_index)
{
}

void Body::Set(const Vec2& w, float m)
{
  position.Set(0.0f, 0.0f);
  rotation = 0.0f;
  velocity.Set(0.0f, 0.0f);
  angularVelocity = 0.0f;
  force.Set(0.0f, 0.0f);
  torque = 0.0f;
  friction = 0.2f;

  width = w;
  mass = m;

  if (mass < FLT_MAX)
  {
    invMass = 1.0f / mass;
    I = mass * (width.x * width.x + width.y * width.y) / 12.0f;
    invI = 1.0f / I;
  }
  else
  {
    invMass = 0.0f;
    I = FLT_MAX;
    invI = 0.0f;
  }
}

void Body::AddForce(const Vec2& f)
{
  force += f;
}

void Body::GetAABB(aabb_t &out) const
{
  const float c = std::abs(cosf(rotation));
  const float s = std::abs(sinf(rotation));

  const Vec2 h = .5f * width;

  const float mx = c * h.x + s * h.y;
  const float my = s * h.x + c * h.y;

  out.min.x = position.x - mx;
  out.min.y = position.y - my;
  out.max.x = position.x + mx;
  out.max.y = position.y + my;
}
