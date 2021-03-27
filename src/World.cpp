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

#include "box2d-lite/World.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/Joint.h"

using namespace b2dl;

using std::vector;
using std::map;
using std::pair;

typedef map<ArbiterKey, Arbiter>::iterator ArbIter;
typedef pair<ArbiterKey, Arbiter> ArbPair;

bool World::accumulateImpulses = true;
bool World::warmStarting = true;
bool World::positionCorrection = true;

void World::Add(Body* body)
{
  bodies.push_back(body);
}

void World::Add(Joint* joint)
{
  joints.push_back(joint);
}

void World::Clear()
{
  bodies.clear();
  joints.clear();
  arbiters.clear();
}

void World::NarrowPhase(Body* bi, Body *bj) {

  // both are static objects
  if (bi->invMass == 0.0f && bj->invMass == 0.0f)
    return;

  Arbiter newArb(bi, bj);

  ArbiterKey key(bi, bj);

  // with no contacts 
  if (newArb.numContacts <= 0)
  {
    arbiters.erase(key);
    return;
  }

  ArbIter iter = arbiters.find(key);
  if (iter == arbiters.end())
  {
    arbiters.insert(ArbPair(key, newArb));
  }
  else
  {
    iter->second.Update(newArb.contacts.data(), newArb.numContacts);
  }
}

void World::BroadPhase()
{
  // O(n^2) broad-phase
  for (size_t i = 0; i < bodies.size(); ++i)
  {
    Body* bi = bodies[i];

    for (size_t j = i + 1; j < bodies.size(); ++j)
    {
      Body* bj = bodies[j];

      // check for a collision between these two
      NarrowPhase(bi, bj);
    }
  }
}

void World::Step(float dt)
{
  const float inv_dt = (dt > 0.0f) ? 1.0f / dt : 0.0f;

  // Determine overlapping bodies and update contact points.
  BroadPhase();

  // Integrate forces
  for (auto &b : bodies)
  {
    assert(b);
    if (b->invMass == 0.0f)
      continue;

    b->velocity += dt * (gravity + b->invMass * b->force);
    b->angularVelocity += dt * b->invI * b->torque;
  }

  // Perform pre-steps
  for (auto &arb : arbiters)
  {
    arb.second.PreStep(inv_dt);
  }

  for (auto &joint : joints)
  {
    assert(joint);
    joint->PreStep(inv_dt);
  }

  // Perform iterations
  for (int i = 0; i < iterations; ++i)
  {
    for (auto &arb : arbiters)
    {
      arb.second.ApplyImpulse();
    }

    for (int j = 0; j < (int)joints.size(); ++j)
    {
      joints[j]->ApplyImpulse();
    }
  }

  // Integrate Velocities
  for (auto &b : bodies)
  {
    assert(b);

    b->position += dt * b->velocity;
    b->rotation += dt * b->angularVelocity;

    b->force.Set(0.0f, 0.0f);
    b->torque = 0.0f;
  }
}
