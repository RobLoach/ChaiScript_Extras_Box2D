# ChaiScript Extras Box2D

Provides [ChaiScript](https://github.com/ChaiScript/ChaiScript) bindings to [Box2D](https://github.com/erincatto/Box2D).

This is a **Proof of Concept**. Any help in getting it up and running would be much appreciated.

## Usage

1. Include the module source...
    ```
    #include "ChaiScript_Extras_Box2D/include/chaiscript/extras/box2d.hpp"
    ```

2. Add the module to the ChaiScript instance...
    ```
    auto box2dlib = chaiscript::extras::box2d::bootstrap();
    chai.add(box2dlib);
    ```

3. Use Box2D in ChaiScript...
    ```
    // TODO: Fill in the API example.
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
