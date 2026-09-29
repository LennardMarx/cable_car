#ifndef SIM_LOOP_H
#define SIM_LOOP_H

#pragma once
#include "UI.h"
#include "cable_car.h"
#include "city.h"
#include "cable_network.h"
#include "event_checks.h"
#include "helper_variables.h"
#include "path_planner.h"
#include "vec2.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <SDL3/SDL.h>

struct context {
  const int FPS = 60;                // set FPS
  const int frameDelay = 1000 / FPS; // delay according to FPS
  Uint64 frameStart;                 // keeps track of time
  Uint64 lastTicks = 0;
  double accumulator = 0; // unsimulated time [s]

  // initial window size (native), in the web the canvas sets the size
  int window_width = 1200;
  int window_height = 800;
  UI ui{window_width, window_height, 20, true};

  // visible world in meters: at least this much, the rest of the window
  // shows more (wide screens: wider, phones: taller)
  const double minViewWidth = 30, minViewHeight = 40;
  double halfWidth = 30, halfHeight = 20; // set by fitView
  double resizeTimer = 0;         // s until the world fits a new window size
  const double resizeDelay = 0.3; // wait for the size to settle

  // the city in the background, cables hang from its buildings
  // (GROWING: canopy of dead trees, PARALLAX: static skyline in layers)
  const CityType cityType = CityType::PARALLAX;
  std::unique_ptr<City> city = makeCity(cityType);
  const int anchorCount = 40; // per 60 x 40 m of visible world

  CableNetwork cableNetwork;
  CableCar cableCar;
  PathPlanner pathPlanner;
  HelperVars helperVars;
  EventChecks eventChecks;

  const double dt = 1.0 / 60; // physics time step
  const int substeps = 30;    // substeps for cables and pod
  unsigned seed = 3;          // world generation

  Vec2 target;            // desired pod position (mouse / arrow keys)
  Vec2 plannedTarget;     // target of the current path
  double replanTimer = 0; // time until the path is planned again
  const double replanInterval = 0.5;

  int frameCount = 0;
};

class SimLoop {
public:
  SimLoop();
  ~SimLoop();

  context ctx;

  void run();
  static void update(context *); // advance the simulation by one time step
  static void render(context *);
  static void buildWorld(context *); // city, cables and pod for ctx->seed
  static void fitView(context *);    // scale and world extents for the window

private:
  static void mainloop(void *arg);
};

#endif
