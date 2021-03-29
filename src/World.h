/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#pragma once

#include <vector>
#include <map>

#include "MathUtils.h"
#include "Arbiter.h"
#include "BVH.h"

namespace b2dl {

struct Body;
struct Joint;

struct World
{
  World(Vec2 gravity, int iterations)
    : gravity(gravity)
    , iterations(iterations)
  {
    bvh.growth = 0.2f;
  }

  void Add(Body* body);
  void Add(Joint* joint);
  void Clear();

  void Step(float dt);

  void BroadPhase();

  void NarrowPhase(Body* a, Body *b);

  std::vector<Body*> bodies;
  std::vector<Joint*> joints;
  std::map<ArbiterKey, Arbiter> arbiters;

  Vec2 gravity;
  int iterations;

  bvh_t bvh;

  static bool useBVH;
  static bool accumulateImpulses;
  static bool warmStarting;
  static bool positionCorrection;
};

}  // namespace b2dl
