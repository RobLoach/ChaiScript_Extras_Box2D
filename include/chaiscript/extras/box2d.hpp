#include <vector>
#include <string>

#include <chaiscript/chaiscript.hpp>

#include "Box2D/Box2D.h"

namespace chaiscript {
  namespace extras {
    namespace box2d {

      /**
       * Register a ChaiScript module with b2Body.
       */
      ModulePtr addb2Body(ModulePtr m = std::make_shared<Module>()) {
        // b2BodyDef
        m->add(user_type<b2BodyDef>(), "b2BodyDef");
        m->add(constructor<b2BodyDef()>(), "b2BodyDef");
        m->add(fun(&b2BodyDef::type), "type");
        m->add(fun(&b2BodyDef::position), "position");
        m->add(fun(&b2BodyDef::angle), "angle");
        m->add(fun(&b2BodyDef::linearVelocity), "linearVelocity");
        m->add(fun(&b2BodyDef::angularVelocity), "angularVelocity");
        m->add(fun(&b2BodyDef::linearDamping), "linearDamping");
        m->add(fun(&b2BodyDef::angularDamping), "angularDamping");
        m->add(fun(&b2BodyDef::allowSleep), "allowSleep");
        m->add(fun(&b2BodyDef::awake), "awake");
        m->add(fun(&b2BodyDef::fixedRotation), "fixedRotation");
        m->add(fun(&b2BodyDef::bullet), "bullet");
        m->add(fun(&b2BodyDef::active), "active");
        m->add(fun(&b2BodyDef::gravityScale), "gravityScale");

        // b2Body
        m->add(user_type<b2Body>(), "b2Body");
        m->add(fun<b2Fixture*, b2Body, const b2FixtureDef*>(&b2Body::CreateFixture), "CreateFixture");
        m->add(fun<b2Fixture*, b2Body, const b2Shape*, float>(&b2Body::CreateFixture), "CreateFixture");
        m->add(fun(&b2Body::DestroyFixture), "DestroyFixture");
        m->add(fun(&b2Body::SetTransform), "SetTransform");
        m->add(fun(&b2Body::GetTransform), "GetTransform");
        m->add(fun(&b2Body::GetPosition), "GetPosition");
        m->add(fun(&b2Body::GetAngle), "GetAngle");
        m->add(fun(&b2Body::GetWorldCenter), "GetWorldCenter");
        m->add(fun(&b2Body::GetLocalCenter), "GetLocalCenter");
        m->add(fun(&b2Body::SetLinearVelocity), "SetLinearVelocity");
        m->add(fun(&b2Body::GetLinearVelocity), "GetLinearVelocity");
        m->add(fun(&b2Body::SetAngularVelocity), "SetAngularVelocity");
        m->add(fun(&b2Body::GetAngularVelocity), "GetAngularVelocity");
        m->add(fun(&b2Body::ApplyForce), "ApplyForce");
        m->add(fun(&b2Body::ApplyForceToCenter), "ApplyForceToCenter");
        m->add(fun(&b2Body::ApplyTorque), "ApplyTorque");
        m->add(fun(&b2Body::ApplyLinearImpulse), "ApplyLinearImpulse");
        m->add(fun(&b2Body::ApplyLinearImpulseToCenter), "ApplyLinearImpulseToCenter");
        m->add(fun(&b2Body::ApplyAngularImpulse), "ApplyAngularImpulse");
        m->add(fun(&b2Body::GetMass), "GetMass");
        m->add(fun(&b2Body::GetInertia), "GetInertia");
        m->add(fun(&b2Body::GetMassData), "GetMassData");
        m->add(fun(&b2Body::SetMassData), "SetMassData");
        m->add(fun(&b2Body::ResetMassData), "ResetMassData");
        m->add(fun(&b2Body::GetWorldPoint), "GetWorldPoint");
        m->add(fun(&b2Body::GetWorldVector), "GetWorldVector");
        m->add(fun(&b2Body::GetLocalPoint), "GetLocalPoint");
        m->add(fun(&b2Body::GetLocalVector), "GetLocalVector");
        m->add(fun(&b2Body::GetLinearVelocityFromWorldPoint), "GetLinearVelocityFromWorldPoint");
        m->add(fun(&b2Body::GetLinearVelocityFromLocalPoint), "GetLinearVelocityFromLocalPoint");
        m->add(fun(&b2Body::GetLinearDamping), "GetLinearDamping");
        m->add(fun(&b2Body::SetLinearDamping), "SetLinearDamping");
        m->add(fun(&b2Body::GetAngularDamping), "GetAngularDamping");
        m->add(fun(&b2Body::SetAngularDamping), "SetAngularDamping");
        m->add(fun(&b2Body::GetGravityScale), "GetGravityScale");
        m->add(fun(&b2Body::SetGravityScale), "SetGravityScale");
        m->add(fun(&b2Body::SetType), "SetType");
        m->add(fun(&b2Body::GetType), "GetType");
        m->add(fun(&b2Body::SetBullet), "SetBullet");
        m->add(fun(&b2Body::IsBullet), "IsBullet");
        m->add(fun(&b2Body::SetSleepingAllowed), "SetSleepingAllowed");
        m->add(fun(&b2Body::IsSleepingAllowed), "IsSleepingAllowed");
        m->add(fun(&b2Body::SetAwake), "SetAwake");
        m->add(fun(&b2Body::IsAwake), "IsAwake");
        m->add(fun(&b2Body::SetActive), "SetActive");
        m->add(fun(&b2Body::IsActive), "IsActive");
        m->add(fun(&b2Body::SetFixedRotation), "SetFixedRotation");
        m->add(fun(&b2Body::IsFixedRotation), "IsFixedRotation");
        m->add(fun<b2Fixture*, b2Body>(&b2Body::GetFixtureList), "GetFixtureList");
        m->add(fun<b2JointEdge*, b2Body>(&b2Body::GetJointList), "GetJointList");
        m->add(fun<b2ContactEdge*, b2Body>(&b2Body::GetContactList), "GetContactList");
        m->add(fun<b2Body*, b2Body>(&b2Body::GetNext), "GetNext");
        /*
        TODO: Figure out a way to pass void* as an argument through ChaiScript.
        m->add(fun(&b2Body::GetUserData), "GetUserData");
        m->add(fun(&b2Body::SetUserData), "SetUserData");
        */
        m->add(fun<b2World*, b2Body>(&b2Body::GetWorld), "GetWorld");
        m->add(fun(&b2Body::Dump), "Dump");

        return m;
      }

      /**
       * Register a ChaiScript module with b2PolygonShape.
       *
       * @see b2PolygonShape.h
       */
      ModulePtr addb2PolygonShape(ModulePtr m = std::make_shared<Module>()) {
        // b2PolygonShape
        m->add(user_type<b2PolygonShape>(), "b2PolygonShape");
        m->add(constructor<b2PolygonShape()>(), "b2PolygonShape");
        m->add(fun(&b2PolygonShape::GetChildCount), "GetChildCount");
        m->add(fun(&b2PolygonShape::Set), "Set");
        // TODO: Add b2BlockAllocator and b2PolygonShape::Clone
        //b2Shape* Clone(b2BlockAllocator* allocator) const override;
        m->add(fun<void, b2PolygonShape, float, float>(&b2PolygonShape::SetAsBox), "SetAsBox");
        m->add(fun<void, b2PolygonShape, float, float, const b2Vec2&, float>(&b2PolygonShape::SetAsBox), "SetAsBox");
        m->add(fun(&b2PolygonShape::TestPoint), "TestPoint");
        m->add(fun(&b2PolygonShape::RayCast), "RayCast");
        m->add(fun(&b2PolygonShape::ComputeAABB), "ComputeAABB");
        m->add(fun(&b2PolygonShape::ComputeMass), "ComputeMass");
        m->add(fun(&b2PolygonShape::Validate), "Validate");

        return m;
      }

      ModulePtr addb2Fixture(ModulePtr m = std::make_shared<Module>()) {
        // b2FixtureDef
        m->add(user_type<b2FixtureDef>(), "b2FixtureDef");
        m->add(constructor<b2FixtureDef()>(), "b2FixtureDef");
        m->add(fun(&b2FixtureDef::shape), "shape");
        m->add(fun(&b2FixtureDef::friction), "friction");
        m->add(fun(&b2FixtureDef::restitution), "restitution");
        m->add(fun(&b2FixtureDef::density), "density");
        m->add(fun(&b2FixtureDef::isSensor), "isSensor");
        m->add(fun(&b2FixtureDef::filter), "filter");

        // b2Filter
        m->add(user_type<b2Filter>(), "b2Filter");
        m->add(constructor<b2Filter()>(), "b2Filter");
        m->add(fun(&b2Filter::categoryBits), "categoryBits");
        m->add(fun(&b2Filter::maskBits), "maskBits");
        m->add(fun(&b2Filter::groupIndex), "groupIndex");

        // b2Fixture
        m->add(user_type<b2Fixture>(), "b2Fixture");
        m->add(fun(&b2Fixture::GetType), "GetType");
        m->add(fun<b2Shape*, b2Fixture>(&b2Fixture::GetShape), "GetShape");
        m->add(fun(&b2Fixture::SetSensor), "SetSensor");
        m->add(fun(&b2Fixture::IsSensor), "IsSensor");
        m->add(fun(&b2Fixture::SetFilterData), "SetFilterData");
        m->add(fun(&b2Fixture::GetFilterData), "GetFilterData");
        m->add(fun(&b2Fixture::Refilter), "Refilter");
        m->add(fun<b2Body*, b2Fixture>(&b2Fixture::GetBody), "GetBody");
        m->add(fun<b2Fixture*, b2Fixture>(&b2Fixture::GetNext), "GetNext");
        //m->add(fun(&b2Fixture::GetUserData), "GetUserData");
        //m->add(fun(&b2Fixture::SetUserData), "SetUserData");
        m->add(fun(&b2Fixture::TestPoint), "TestPoint");
        m->add(fun(&b2Fixture::RayCast), "RayCast");
        m->add(fun(&b2Fixture::GetMassData), "GetMassData");
        m->add(fun(&b2Fixture::SetDensity), "SetDensity");
        m->add(fun(&b2Fixture::GetDensity), "GetDensity");
        m->add(fun(&b2Fixture::GetFriction), "GetFriction");
        m->add(fun(&b2Fixture::SetFriction), "SetFriction");
        m->add(fun(&b2Fixture::GetRestitution), "GetRestitution");
        m->add(fun(&b2Fixture::SetRestitution), "SetRestitution");
        m->add(fun(&b2Fixture::GetAABB), "GetAABB");
        m->add(fun(&b2Fixture::Dump), "Dump");

        return m;
      }

      /**
       * Register a ChaiScript module with b2Math.
       *
       * @see b2World.h
       */
      ModulePtr addb2World(ModulePtr m = std::make_shared<Module>()) {
        // b2World
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
        addb2Body(m);
        addb2PolygonShape(m);
        addb2Fixture(m);
        addb2World(m);
        addb2Math(m);

        return m;
      }
    }
  }
}
