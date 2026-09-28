#ifndef HELPER_VARS_H
#define HELPER_VARS_H

#pragma once
#include "vec2.h"
#include <vector>

class HelperVars {
public:
  void toggleQuit() { quit = !quit; }
  bool getQuit() { return quit; }

  void toggleReset() { reset = !reset; }
  bool getReset() { return reset; }

  void toggleRegenerate() { regenerate = !regenerate; }
  bool getRegenerate() { return regenerate; }

  void togglePause() { pause = !pause; }
  bool getPause() { return pause; }

  void toggleTrajOn() { trajOn = !trajOn; };
  bool getTrajOn() { return trajOn; }

  void toggleDebug() { debug = !debug; }
  bool getDebug() { return debug; }

  void togglePathOn() { pathOn = !pathOn; }
  bool getPathOn() { return pathOn; }

  void toggleScreenshot() { screenshot = !screenshot; }
  bool getScreenshot() { return screenshot; }

  std::vector<Vec2> &getTrajectory() { return trajectory; }

private:
  static inline bool quit = false;
  static inline bool reset = false;
  static inline bool regenerate = false;
  static inline bool pause = false;
  static inline bool trajOn = false;
  static inline bool debug = false;
  static inline bool screenshot = false;
  static inline bool pathOn = false; // path finding instead of direct line
  static inline std::vector<Vec2> trajectory;
};

#endif
