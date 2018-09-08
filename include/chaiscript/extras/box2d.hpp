#include <vector>
#include <string>

#include <chaiscript/chaiscript.hpp>

#include "Box2D/Box2D.h"

namespace chaiscript {
  namespace extras {
    namespace box2d {

      /**
       * Register a ChaiScript module with b2Math.
       *
       * @see b2World.h
       */
      ModulePtr addb2World(ModulePtr m = std::make_shared<Module>()) {
        m->add(user_type<b2World>(), "b2World");
        m->add(constructor<b2World(const b2Vec2&)>(), "b2World");
        m->add(fun(&b2World::SetDestructionListener), "SetDestructionListener");
        m->add(fun(&b2World::SetContactFilter), "SetContactFilter");
        m->add(fun(&b2World::SetContactListener), "SetContactListener");
        m->add(fun(&b2World::SetDebugDraw), "SetDebugDraw");
        m->add(fun(&b2World::CreateBody), "CreateBody");
        m->add(fun(&b2World::DestroyBody), "DestroyBody");
        m->add(fun(&b2World::CreateJoint), "CreateJoint");
        m->add(fun(&b2World::DestroyJoint), "DestroyJoint");
        m->add(fun(&b2World::Step), "Step");
        m->add(fun(&b2World::ClearForces), "ClearForces");
        m->add(fun(&b2World::DrawDebugData), "DrawDebugData");
        m->add(fun(&b2World::QueryAABB), "QueryAABB");
        m->add(fun(&b2World::RayCast), "RayCast");
        /*
        TODO: Add function overrides.
        b2Body* GetBodyList();
        const b2Body* GetBodyList() const;
        b2Joint* GetJointList();
        const b2Joint* GetJointList() const;
        b2Contact* GetContactList();
        const b2Contact* GetContactList() const;
        */
        m->add(fun(&b2World::SetAllowSleeping), "SetAllowSleeping");
        m->add(fun(&b2World::GetAllowSleeping), "GetAllowSleeping");
        m->add(fun(&b2World::SetWarmStarting), "SetWarmStarting");
        m->add(fun(&b2World::GetWarmStarting), "GetWarmStarting");
        m->add(fun(&b2World::SetContinuousPhysics), "SetContinuousPhysics");
        m->add(fun(&b2World::GetContinuousPhysics), "GetContinuousPhysics");
        m->add(fun(&b2World::SetSubStepping), "SetSubStepping");
        m->add(fun(&b2World::GetSubStepping), "GetSubStepping");
        m->add(fun(&b2World::GetProxyCount), "GetProxyCount");
        m->add(fun(&b2World::GetBodyCount), "GetBodyCount");
        m->add(fun(&b2World::GetJointCount), "GetJointCount");
        m->add(fun(&b2World::GetContactCount), "GetContactCount");
        m->add(fun(&b2World::GetTreeHeight), "GetTreeHeight");
        m->add(fun(&b2World::GetTreeBalance), "GetTreeBalance");
        m->add(fun(&b2World::GetTreeQuality), "GetTreeQuality");
        m->add(fun(&b2World::SetGravity), "SetGravity");
        m->add(fun(&b2World::GetGravity), "GetGravity");
        m->add(fun(&b2World::IsLocked), "IsLocked");
        m->add(fun(&b2World::SetAutoClearForces), "SetAutoClearForces");
        m->add(fun(&b2World::GetAutoClearForces), "GetAutoClearForces");
        m->add(fun(&b2World::ShiftOrigin), "ShiftOrigin");
        m->add(fun(&b2World::GetContactManager), "GetContactManager");
        m->add(fun(&b2World::GetProfile), "GetProfile");
        m->add(fun(&b2World::Dump), "Dump");

        return m;
      }

      /**
       * Register a ChaiScript module with b2Math.
       */
      ModulePtr addb2Math(ModulePtr m = std::make_shared<Module>()) {
        m->add(user_type<b2Vec2>(), "b2Vec2");
        m->add(constructor<b2Vec2()>(), "b2Vec2");
        m->add(constructor<b2Vec2(float, float)>(), "b2Vec2");
        m->add(fun(&b2Vec2::x), "x");
        m->add(fun(&b2Vec2::y), "y");
        m->add(fun(&b2Vec2::Set), "Set");
        m->add(fun(&b2Vec2::SetZero), "SetZero");
        m->add(fun(&b2Vec2::Length), "Length");
        m->add(fun(&b2Vec2::LengthSquared), "LengthSquared");
        m->add(fun(&b2Vec2::Normalize), "Normalize");
        m->add(fun(&b2Vec2::IsValid), "IsValid");
        m->add(fun(&b2Vec2::Skew), "Skew");

        return m;
      }

      /**
       * Bootstrap a ChaiScript module with Box2D support.
       */
      ModulePtr bootstrap(ModulePtr m = std::make_shared<Module>())
      {
        addb2Math(m);
        addb2World(m);

        return m;
      }
    }
  }
}
