#include "../include/event_checks.h"

void EventChecks::checkEvents(HelperVars &_helperVars, CableCar &_cableCar,
                              CableNetwork &_cableNetwork, UI &_ui,
                              Vec2 &_target) {
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
    case SDL_EVENT_QUIT:
      _helperVars.toggleQuit();
      break;
    // the target follows the mouse
    case SDL_EVENT_MOUSE_MOTION:
      _target = _ui.toWorld(event.motion.x, event.motion.y);
      break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
      _target = _ui.toWorld(event.button.x, event.button.y);
      break;
    case SDL_EVENT_KEY_DOWN:
      if (event.key.repeat)
        break;
      switch (event.key.key) {
      case SDLK_ESCAPE:
      case SDLK_Q:
        _helperVars.toggleQuit();
        break;
      case SDLK_C:
        _cableCar.toggleController();
        break;
      case SDLK_R:
        _helperVars.toggleReset();
        break;
      case SDLK_G:
        _helperVars.toggleRegenerate();
        break;
      case SDLK_T:
        _helperVars.toggleTrajOn();
        _helperVars.getTrajectory().clear();
        break;
      case SDLK_D:
        _helperVars.toggleDebug();
        break;
      case SDLK_W:
        _cableNetwork.toggleWind();
        break;
      case SDLK_P:
        _helperVars.toggleScreenshot();
        break;
      case SDLK_SPACE:
        _helperVars.togglePause();
        break;
      default:
        break;
      }
      break;
    default:
      break;
    }
  }
}

// arrow keys move the target
void EventChecks::moveTarget(Vec2 &_target, double _dt) {
  const bool *keys = SDL_GetKeyboardState(nullptr);
  Vec2 dir;
  if (keys[SDL_SCANCODE_LEFT])
    dir.x -= 1;
  if (keys[SDL_SCANCODE_RIGHT])
    dir.x += 1;
  if (keys[SDL_SCANCODE_UP])
    dir.y += 1;
  if (keys[SDL_SCANCODE_DOWN])
    dir.y -= 1;
  _target += dir.normalized() * targetSpeed * _dt;
}
