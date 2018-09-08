#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file

#include "catch.hpp"

#include <string>
#include <chaiscript/chaiscript.hpp>
#include <Box2D/Box2D.h>

/**
 * Load the Box2D library.
 */
#include "../include/chaiscript/extras/box2d.hpp"

TEST_CASE( "Box2D functions work", "[box2d]" ) {

  auto box2dlib = chaiscript::extras::box2d::bootstrap();
  
  // Create the ChaiScript environment.
  chaiscript::ChaiScript chai;

  // Add the library to the ChaiScript instance.
  chai.add(box2dlib);

  // Define the gravity vector.
  chai.eval(R""(
    global gravity = b2Vec2(0.0f, -10.0f);
  )"");

  CHECK(chai.eval<float>("map.y") == -10.0f);
}
