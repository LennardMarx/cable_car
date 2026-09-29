#!/bin/bash

emcc src/*.cpp -std=c++17 -s WASM=1 -s USE_SDL=3 -s ALLOW_MEMORY_GROWTH=1 -O3 -o cable_car.js
g++ src/*.cpp -std=c++17 -I. -Iinclude/ -lSDL3 -O2 -o bin/cable_car

# skyline growth demo
emcc demo/growth_demo.cpp src/UI.cpp src/skyline.cpp -std=c++17 -Iinclude/ -s WASM=1 -s USE_SDL=3 -s ALLOW_MEMORY_GROWTH=1 -O3 -o growth_demo.js
g++ demo/growth_demo.cpp src/UI.cpp src/skyline.cpp -std=c++17 -Iinclude/ -lSDL3 -O2 -o bin/growth_demo
