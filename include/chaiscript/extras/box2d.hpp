#include <vector>
#include <string>

#include <chaiscript/chaiscript.hpp>

#include "Box2D/Box2D.h"

namespace chaiscript {
  namespace extras {
    namespace box2d {

      /**
       * Register a ChaiScript module with b2Vec2 support.
       */
      ModulePtr addb2Vec2(ModulePtr m = std::make_shared<Module>()) {
        m->add(user_type<b2Vec2>(), "b2Vec2");
        m->add(constructor<b2Vec2()>(), "b2Vec2");
        m->add(constructor<b2Vec2(float, float)>(), "b2Vec2");
        m->add(fun(&b2Vec2::x), "x");
        m->add(fun(&b2Vec2::y), "y");
        m->add(fun(&b2Vec2::set), "set");
        m->add(fun(&b2Vec2::setZero), "setZero");
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
        addb2Vec2(m);

        return m;
      }
    }
  }
}
