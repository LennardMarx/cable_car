#!/bin/bash
# Build the cable car project on Windows via WSL (Ubuntu).
#
# Builds two targets:
#   1. Native WSL binary        -> bin/cable_car          (run with WSLg)
#   2. WebAssembly (Emscripten) -> cable_car.js/.wasm     (for the website)
#   3. Skyline growth demo      -> bin/growth_demo, growth_demo.js/.wasm
#
# Usage, from a WSL/Ubuntu terminal in this directory:
#   ./build_windows.sh           # build both
#   ./build_windows.sh native    # native binary only
#   ./build_windows.sh wasm      # emscripten build only
#   ./build_windows.sh demo      # growth demo (native + wasm)
#
# SDL3 is not packaged for Ubuntu 24.04, build it once into a local prefix:
#   git clone --depth 1 --branch release-3.2.24 https://github.com/libsdl-org/SDL.git
#   cmake -S SDL -B SDL/build -DCMAKE_INSTALL_PREFIX=$HOME/Software/SDL3/install
#   cmake --build SDL/build -j8 && cmake --install SDL/build
#
# Override locations with:
#   SDL3_DIR=/path/to/sdl3/prefix EMSDK_ENV=/path/to/emsdk_env.sh ./build_windows.sh

set -e

# Always operate relative to this script's location, regardless of cwd.
cd "$(dirname "$0")"

SDL3_DIR="${SDL3_DIR:-$HOME/Software/SDL3/install}"
EMSDK_ENV="${EMSDK_ENV:-$HOME/Software/emsdk/emsdk_env.sh}"

build_native() {
  echo ">> Building native binary -> bin/cable_car"
  mkdir -p bin
  g++ src/*.cpp -std=c++17 -Iinclude/ -I"$SDL3_DIR/include" \
    -L"$SDL3_DIR/lib" -Wl,-rpath,"$SDL3_DIR/lib" -lSDL3 -O2 -Wall \
    -o bin/cable_car
  echo ">> Done. Run it with: ./bin/cable_car"
}

build_wasm() {
  echo ">> Building WebAssembly -> cable_car.js/.wasm"
  if [ ! -f "$EMSDK_ENV" ]; then
    echo "!! emsdk not found at: $EMSDK_ENV" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  source "$EMSDK_ENV" >/dev/null 2>&1
  emcc src/*.cpp -std=c++17 -s WASM=1 -s USE_SDL=3 -s ALLOW_MEMORY_GROWTH=1 -O3 -o cable_car.js
  echo ">> Done. Serve from the website root and open projects/cable_car/cable_car.html"
}

# skyline growth demo, shares UI and skyline with the game
DEMO_SOURCES="demo/growth_demo.cpp src/UI.cpp src/skyline.cpp"

build_demo() {
  echo ">> Building growth demo -> bin/growth_demo, growth_demo.js/.wasm"
  mkdir -p bin
  g++ $DEMO_SOURCES -std=c++17 -Iinclude/ -I"$SDL3_DIR/include"     -L"$SDL3_DIR/lib" -Wl,-rpath,"$SDL3_DIR/lib" -lSDL3 -O2 -Wall     -o bin/growth_demo
  if [ -f "$EMSDK_ENV" ]; then
    # shellcheck disable=SC1090
    source "$EMSDK_ENV" >/dev/null 2>&1
    emcc $DEMO_SOURCES -std=c++17 -Iinclude/ -s WASM=1 -s USE_SDL=3 -s ALLOW_MEMORY_GROWTH=1 -O3       -o growth_demo.js
  fi
  echo ">> Done. Run it with: ./bin/growth_demo"
}

case "${1:-all}" in
  native) build_native ;;
  wasm)   build_wasm ;;
  demo)   build_demo ;;
  all)    build_native; build_wasm; build_demo ;;
  *) echo "Usage: $0 [native|wasm|demo|all]" >&2; exit 1 ;;
esac

echo ">> Build complete."
