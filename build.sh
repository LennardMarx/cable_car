#!/bin/bash

emcc src/*.cpp -std=c++17 -s WASM=1 -s USE_SDL=3 -O3 -o cable_car.js
g++ src/*.cpp -std=c++17 -I. -Iinclude/ -lSDL3 -O2 -o bin/cable_car
