#ifndef UI_H
#define UI_H

#pragma once
#include "vec2.h"
#include <SDL3/SDL.h>
#include <cstdint>
#include <functional>
#include <vector>

// window and drawing helpers
// world coordinates: meters, origin in the window center, y pointing up
class UI {
public:
  const int sizeX;
  const int sizeY;
  const double scale; // pixels per meter

  UI(int, int, double);
  ~UI();

  void clear();
  void present();
  void setDrawColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a);
  void setAlpha(uint8_t a); // gruv-light with the given alpha

  SDL_FPoint toScreen(const Vec2 &);
  Vec2 toWorld(double, double);

  void drawLine(const Vec2 &, const Vec2 &);
  void drawThickLine(const Vec2 &, const Vec2 &, double); // width in pixels
  void drawCircle(const Vec2 &, double);                   // radius in meters
  void fillCircle(const Vec2 &, double);                   // radius in meters
  void fillPolygon(const std::vector<Vec2> &);             // convex polygon
  // any closed polygon (even-odd), calls back with the pixel spans per row
  void scanPolygon(const std::vector<Vec2> &,
                   const std::function<void(int, float, float)> &);
  void drawSpan(int, float, float);
  void fillQuads(const std::vector<Vec2> &); // 4 corners per quad, one batch
  void drawTrajectory(std::vector<Vec2> &, int);

  bool saveScreenshot(const char *);

  SDL_Renderer *&getRenderer(); // pointer reference to the renderer
  SDL_Window *getWindow();      // pointer to the window

private:
  void initialize(int sizeX, int sizeY);
  SDL_FColor getFColor();

private:
  SDL_Window *window = nullptr;     // create window pointer
  SDL_Renderer *renderer = nullptr; // create renderer pointer
  SDL_Color drawColor = {249, 245, 215, 255};
  std::vector<std::vector<float>> rowCrossings; // reused by scanPolygon
};

#endif
