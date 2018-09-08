# ChaiScript Extras Box2D [![Build Status](https://travis-ci.org/RobLoach/ChaiScript_Extras_Box2D.svg?branch=master)](https://travis-ci.org/RobLoach/ChaiScript_Extras_Box2D)

Provides [ChaiScript](https://github.com/ChaiScript/ChaiScript) bindings to [Box2D](https://github.com/erincatto/Box2D).

This is currently just a proof of concept. Any assistance in getting this up and running would be much appreciated.

## Usage

1. Include the module source...
    ```cpp
    #include "ChaiScript_Extras_Box2D/include/chaiscript/extras/box2d.hpp"
    ```

2. Add the module to the ChaiScript instance...
    ```cpp
    auto box2dlib = chaiscript::extras::box2d::bootstrap();
    chai.add(box2dlib);
    ```

3. Use Box2D in ChaiScript...
    ```js
    // Define the gravity vector.
    var gravity = b2Vec2(0.0f, -10.0f)

    // Construct a world object, which will hold and simulate the rigid bodies.
    var world = b2World(gravity)

    // Define the ground body.
    var groundBodyDef = b2BodyDef()
    groundBodyDef.position.Set(0.0f, -10.0f)

    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    var groundBody = world.CreateBody(groundBodyDef)

    // Define the ground box shape.
    var groundBox = b2PolygonShape()

    // The extents are the half-widths of the box.
    groundBox.SetAsBox(50.0f, 10.0f)

    // Add the ground fixture to the ground body.
    groundBody.CreateFixture(groundBox, 0.0f)

    // Define the dynamic body. We set its position and call the body factory.
    var bodyDef = b2BodyDef()
    bodyDef.type = b2_dynamicBody
    bodyDef.position.Set(0.0f, 4.0f)
    var body = world.CreateBody(bodyDef)

    // Define another box shape for our dynamic body.
    var dynamicBox = b2PolygonShape()
    dynamicBox.SetAsBox(1.0f, 1.0f)

    // Define the dynamic body fixture.
    var fixtureDef = b2FixtureDef()
    fixtureDef.shape = dynamicBox

    // Set the box density to be non-zero, so it will be dynamic.
    fixtureDef.density = 1.0f

    // Override the default friction.
    fixtureDef.friction = 0.3f

    // Add the shape to the body.
    body.CreateFixture(fixtureDef)

    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    var timeStep = 1.0f / 60.0f
    var velocityIterations = 6
    var positionIterations = 2

    // This is our little game loop.
    for (var i = 0; i < 60; ++i) {
        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        world.Step(timeStep, velocityIterations, positionIterations)

        // Now print the position and angle of the body.
        var position = body.GetPosition()
        var angle = body.GetAngle()

        print("X: " + to_string(position.x))
        print("Y: " + to_string(position.y))
        print("A: " + to_string(angle))
    }
    ```

## Development

```
git submodule update --init
mkdir build
cd build
cmake ..
make
make test
```

## License

[MIT](LICENSE)
