build:
	mkdir -p build

vendor/Box2D/LICENSE:
	git submodule update --init

build/Makefile: build vendor/Box2D/LICENSE
	cd build && cmake ..

compile: build/Makefile
	make -C build

compile-test: compile
	make -C build test

build/box2d_test: compile-test
	./build/box2d_test

test: build/box2d_test

clean:
	rm -rf build
