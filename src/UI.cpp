#include "../include/UI.h"
#include <algorithm>

UI::UI(int sizeX, int sizeY, double _scale, bool _resizable)
    : sizeX(sizeX), sizeY(sizeY), scale(_scale) {
  initialize(sizeX, sizeY, _resizable);
  fitWindow(); // in the web the canvas brings its own size
}

UI::~UI() {
  if (renderer)
    SDL_DestroyRenderer(renderer);
  if (window)
    SDL_DestroyWindow(window);
  SDL_Quit();
}

void UI::clear() {
  setDrawColor(69, 133, 136, 255); // gruv-blue
  SDL_RenderClear(renderer);
  setDrawColor(249, 245, 215, 255); // gruv-light (lightest)
}

void UI::present() { SDL_RenderPresent(renderer); }

bool UI::fitWindow() {
  int w, h;
  if (!SDL_GetCurrentRenderOutputSize(renderer, &w, &h) || w <= 0 || h <= 0 ||
      (w == sizeX && h == sizeY))
    return false;
  sizeX = w;
  sizeY = h;
  return true;
}

void UI::setDrawColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
  drawColor = {r, g, b, a};
  SDL_SetRenderDrawColor(renderer, r, g, b, a);
}

void UI::setAlpha(uint8_t a) { setDrawColor(249, 245, 215, a); }

SDL_FColor UI::getFColor() {
  return {drawColor.r / 255.0f, drawColor.g / 255.0f, drawColor.b / 255.0f,
          drawColor.a / 255.0f};
}

// world (meters, y up) -> screen (pixels, y down)
SDL_FPoint UI::toScreen(const Vec2 &_p) {
  return {(float)(sizeX / 2 + (_p.x + offset.x) * scale),
          (float)(sizeY / 2 - (_p.y + offset.y) * scale)};
}

// screen -> world
Vec2 UI::toWorld(double _x, double _y) {
  return Vec2((_x - sizeX / 2) / scale, -(_y - sizeY / 2) / scale) - offset;
}

// function to draw line between two points
void UI::drawLine(const Vec2 &_a, const Vec2 &_b) {
  SDL_FPoint a = toScreen(_a);
  SDL_FPoint b = toScreen(_b);
  SDL_RenderLine(renderer, a.x, a.y, b.x, b.y);
}

// line as a quad with a given width in pixels
void UI::drawThickLine(const Vec2 &_a, const Vec2 &_b, double _width) {
  if (_width <= 1.5) {
    drawLine(_a, _b);
    return;
  }
  SDL_FPoint a = toScreen(_a);
  SDL_FPoint b = toScreen(_b);
  float dx = b.x - a.x, dy = b.y - a.y;
  float len = std::sqrt(dx * dx + dy * dy);
  if (len < 1e-4f)
    return;
  // normal offset of half the width
  float nx = -dy / len * (float)_width / 2;
  float ny = dx / len * (float)_width / 2;

  SDL_FColor c = getFColor();
  SDL_Vertex v[4] = {{{a.x + nx, a.y + ny}, c, {0, 0}},
                     {{b.x + nx, b.y + ny}, c, {0, 0}},
                     {{b.x - nx, b.y - ny}, c, {0, 0}},
                     {{a.x - nx, a.y - ny}, c, {0, 0}}};
  int indices[6] = {0, 1, 2, 0, 2, 3};
  SDL_RenderGeometry(renderer, nullptr, v, 4, indices, 6);
}

void UI::drawCircle(const Vec2 &_c, double _r) {
  const int n = 32;
  for (int i = 0; i < n; ++i) {
    double a1 = 2 * M_PI * i / n, a2 = 2 * M_PI * (i + 1) / n;
    drawLine(_c + Vec2(cos(a1), sin(a1)) * _r, _c + Vec2(cos(a2), sin(a2)) * _r);
  }
}

void UI::fillCircle(const Vec2 &_c, double _r) {
  const int n = 16;
  std::vector<Vec2> points;
  for (int i = 0; i < n; ++i) {
    double a = 2 * M_PI * i / n;
    points.push_back(_c + Vec2(cos(a), sin(a)) * _r);
  }
  fillPolygon(points);
}

// triangle fan from the first vertex (convex polygons only)
void UI::fillPolygon(const std::vector<Vec2> &_points) {
  if (_points.size() < 3)
    return;
  SDL_FColor c = getFColor();
  std::vector<SDL_Vertex> vertices;
  std::vector<int> indices;
  for (const Vec2 &p : _points)
    vertices.push_back({toScreen(p), c, {0, 0}});
  for (int i = 1; i + 1 < (int)_points.size(); ++i) {
    indices.push_back(0);
    indices.push_back(i);
    indices.push_back(i + 1);
  }
  SDL_RenderGeometry(renderer, nullptr, vertices.data(), vertices.size(),
                     indices.data(), indices.size());
}

// scanline fill: crossings of every edge with the pixel row centers,
// spans between pairs of crossings (even-odd rule)
void UI::scanPolygon(const std::vector<Vec2> &_points,
                     const std::function<void(int, float, float)> &_span) {
  if (_points.size() < 3)
    return;
  rowCrossings.resize(sizeY);
  int minRow = sizeY, maxRow = -1;
  for (size_t k = 0; k < _points.size(); ++k) {
    SDL_FPoint a = toScreen(_points[k]);
    SDL_FPoint b = toScreen(_points[(k + 1) % _points.size()]);
    if (a.y == b.y)
      continue;
    float y0 = std::min(a.y, b.y), y1 = std::max(a.y, b.y);
    int r0 = std::max(0, (int)std::ceil(y0 - 0.5f));
    int r1 = std::min(sizeY - 1, (int)std::ceil(y1 - 0.5f) - 1);
    for (int r = r0; r <= r1; ++r) {
      float yc = r + 0.5f;
      rowCrossings[r].push_back(a.x + (yc - a.y) * (b.x - a.x) / (b.y - a.y));
    }
    minRow = std::min(minRow, r0);
    maxRow = std::max(maxRow, r1);
  }
  for (int r = minRow; r <= maxRow; ++r) {
    std::vector<float> &xs = rowCrossings[r];
    std::sort(xs.begin(), xs.end());
    for (size_t k = 0; k + 1 < xs.size(); k += 2)
      _span(r, xs[k], xs[k + 1]);
    xs.clear();
  }
}

// many quads (4 corners each, in order around the quad) in one call
void UI::fillQuads(const std::vector<Vec2> &_corners) {
  if (_corners.size() < 4)
    return;
  SDL_FColor c = getFColor();
  std::vector<SDL_Vertex> vertices;
  std::vector<int> indices;
  vertices.reserve(_corners.size());
  indices.reserve(_corners.size() / 4 * 6);
  for (size_t q = 0; q + 3 < _corners.size(); q += 4) {
    int v = vertices.size();
    for (int k = 0; k < 4; ++k)
      vertices.push_back({toScreen(_corners[q + k]), c, {0, 0}});
    for (int k : {0, 1, 2, 0, 2, 3})
      indices.push_back(v + k);
  }
  SDL_RenderGeometry(renderer, nullptr, vertices.data(), vertices.size(),
                     indices.data(), indices.size());
}

void UI::drawSpan(int _row, float _x0, float _x1) {
  SDL_RenderLine(renderer, _x0, _row + 0.5f, _x1, _row + 0.5f);
}

// draw trajectory (intensity dependend on recency)
void UI::drawTrajectory(std::vector<Vec2> &_trajectory, int _length) {
  for (int i = 1; i < (int)_trajectory.size(); i++) {
    setAlpha(i * 255 / _length);
    drawLine(_trajectory[i - 1], _trajectory[i]);
  }
  // only draw last X positions
  if ((int)_trajectory.size() > _length) {
    _trajectory.erase(_trajectory.begin(), _trajectory.end() - _length);
  }
}

bool UI::saveScreenshot(const char *_path) {
  SDL_Surface *surface = SDL_RenderReadPixels(renderer, nullptr);
  if (!surface)
    return false;
  bool ok = SDL_SaveBMP(surface, _path);
  SDL_DestroySurface(surface);
  return ok;
}

SDL_Renderer *&UI::getRenderer() // pointer reference to the renderer
{
  return renderer;
}
SDL_Window *UI::getWindow() // pointer to the window
{
  return window;
}

// initializing the UI
void UI::initialize(int sizeX, int sizeY, bool _resizable) {
  SDL_Init(SDL_INIT_VIDEO);

  // Create a Window
  window = SDL_CreateWindow("Lennard Marx", sizeX, sizeY,
                            _resizable ? SDL_WINDOW_RESIZABLE : 0);
  renderer = SDL_CreateRenderer(window, nullptr);
  SDL_SetRenderVSync(renderer, 1);

  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
  // place window in middle of screen
  SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
}
