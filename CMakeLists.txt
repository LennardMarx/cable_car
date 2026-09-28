cmake_minimum_required(VERSION 3.16)
project(CableCar)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add source and include directories
file(GLOB_RECURSE SOURCES src/*.cpp)
include_directories(include)

# skyline growth demo (standalone for now)
set(DEMO_SOURCES demo/growth_demo.cpp src/UI.cpp src/skyline.cpp)

# Option to toggle between Emscripten and g++
option(USE_EMSCRIPTEN "Build with Emscripten" OFF)

if (USE_EMSCRIPTEN)
    # Emscripten setup
    set(CMAKE_CXX_COMPILER emcc)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s WASM=1 -s USE_SDL=3 -O3")
    set(CMAKE_EXECUTABLE_SUFFIX ".js")

    # Output target for Emscripten
    add_executable(cable_car ${SOURCES})
    set_target_properties(cable_car PROPERTIES
        OUTPUT_NAME "cable_car"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    add_executable(growth_demo ${DEMO_SOURCES})
    set_target_properties(growth_demo PROPERTIES
        OUTPUT_NAME "growth_demo"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
else()
    # Native build setup
    find_package(SDL3 REQUIRED CONFIG)

    # Output target for native build
    add_executable(cable_car ${SOURCES})
    target_link_libraries(cable_car SDL3::SDL3)
    set_target_properties(cable_car PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
    )
    add_executable(growth_demo ${DEMO_SOURCES})
    target_link_libraries(growth_demo SDL3::SDL3)
    set_target_properties(growth_demo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
    )
endif()
