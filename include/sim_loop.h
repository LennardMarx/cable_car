#ifndef SIM_LOOP_H
#define SIM_LOOP_H

#pragma once
#include "UI.h"
#include "cable_car.h"
#include "cable_network.h"
#include "event_checks.h"
#include "helper_variables.h"
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

  int window_width = 1200;
  int window_height = 800;
  double pixelsPerMeter = 20;
  UI ui{window_width, window_height, pixelsPerMeter};

  // visible world in meters
  double halfWidth = window_width / pixelsPerMeter / 2;
  double halfHeight = window_height / pixelsPerMeter / 2;

  CableNetwork cableNetwork;
  CableCar cableCar;
  HelperVars helperVars;
  EventChecks eventChecks;

  const double dt = 1.0 / 60; // physics time step
  const int substeps = 30;    // substeps for cables and pod
  unsigned seed = 3;          // world generation

  Vec2 target; // desired pod position (mouse / arrow keys)

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

private:
  static void mainloop(void *arg);
};

#endif
