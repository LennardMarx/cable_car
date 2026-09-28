#ifndef EVENT_CHECKS_H
#define EVENT_CHECKS_H

#include "UI.h"
#include "cable_car.h"
#include "cable_network.h"
#include "helper_variables.h"
#include "vec2.h"
#include <SDL3/SDL.h>

class EventChecks {
public:
  void checkEvents(HelperVars &, CableCar &, CableNetwork &, UI &, Vec2 &);
  void moveTarget(Vec2 &, double); // arrow keys

private:
  SDL_Event event;
  const double targetSpeed = 9; // m/s
};

#endif
