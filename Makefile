build:
	mkdir -p build

vendor/Box2D/LICENSE:
	git submodule update --init

build/Makefile: build vendor/Box2D/LICENSE
	cd build && cmake ..

compile: build/Makefile
	make -C build

test: compile
	make -C build test

clean:
	rm -rf build
