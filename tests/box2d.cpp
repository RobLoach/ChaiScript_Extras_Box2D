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

  // b2Vec2
  chai.eval(R""(
    // Define the gravity vector.
    var gravity = b2Vec2(0.0f, -10.0f);
  )"");
  CHECK(chai.eval<float>("gravity.y") == -10.0f);

  // b2World
  chai.eval(R""(
    // Define the gravity vector.
    var world = b2World(gravity);
  )"");
  CHECK(chai.eval<bool>("world.GetSubStepping") == false);

  // b2BodyDef
  chai.eval(R""(
    // Define the ground body.
    var groundBodyDef = b2BodyDef()
    groundBodyDef.position.Set(0.0f, -10.0f)
  )"");
  CHECK(chai.eval<bool>("groundBodyDef.awake") == true);

  // b2Body
  chai.eval(R""(
    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    var groundBody = world.CreateBody(groundBodyDef)
  )"");
  CHECK(chai.eval<bool>("groundBody.IsBullet()") == false);

  // b2PolygonShape
  chai.eval(R""(
    // Define the ground box shape.
    var groundBox = b2PolygonShape()

    // The extents are the half-widths of the box.
    groundBox.SetAsBox(50.0f, 10.0f)
  )"");
  CHECK(chai.eval<bool>("groundBox.Validate()") == true);

  // b2Fixture
  chai.eval(R""(
    // Add the ground fixture to the ground body.
    var newFixture = groundBody.CreateFixture(groundBox, 0.0f)
  )"");
  CHECK(chai.eval<float>("newFixture.GetDensity()") == 0.0f);

  // b2BodyType
  chai.eval(R""(
    // Define the dynamic body. We set its position and call the body factory.
    var bodyDef = b2BodyDef()
    bodyDef.type = b2_dynamicBody
    bodyDef.position.Set(0.0f, 4.0f)
    //var body = world.CreateBody(bodyDef)
  )"");
  CHECK(chai.eval<b2BodyType>("bodyDef.type") == b2_dynamicBody);

}
