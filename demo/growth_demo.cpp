// standalone demo: skyscrapers morphing into tree-like crowns
#include "../include/UI.h"
#include "../include/skyline.h"
#include <SDL3/SDL.h>
#include <string>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

struct context {
  const int FPS = 60;                // set FPS
  const int frameDelay = 1000 / FPS; // delay according to FPS
  Uint64 frameStart;

  int window_width = 1200;
  int window_height = 800;
  double pixelsPerMeter = 3;
  UI ui{window_width, window_height, pixelsPerMeter};
  double halfWidth = window_width / pixelsPerMeter / 2;
  double halfHeight = window_height / pixelsPerMeter / 2;

  Skyline skyline;
  unsigned seed = 1;

  bool quit = false;
  bool pause = false;
  bool debug = false;
  int stepsPerFrame = 3;  // growth speed
  int growthDelay = 90;   // frames showing the original skyline
  int frameCount = 0;
  int growthFrames = 0;
};

static void restart(context *ctx) {
  ctx->skyline.generate(ctx->seed, ctx->halfWidth, ctx->halfHeight);
  ctx->growthFrames = 0;
}

static void checkEvents(context *ctx) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_EVENT_QUIT)
      ctx->quit = true;
    if (event.type != SDL_EVENT_KEY_DOWN || event.key.repeat)
      continue;
    switch (event.key.key) {
    case SDLK_ESCAPE:
    case SDLK_Q:
      ctx->quit = true;
      break;
    case SDLK_SPACE:
      ctx->pause = !ctx->pause;
      break;
    case SDLK_R:
      restart(ctx);
      break;
    case SDLK_G:
      ctx->seed++;
      restart(ctx);
      break;
    case SDLK_D:
      ctx->debug = !ctx->debug;
      break;
    case SDLK_UP:
      ctx->stepsPerFrame = std::min(ctx->stepsPerFrame * 2, 32);
      break;
    case SDLK_DOWN:
      ctx->stepsPerFrame = std::max(ctx->stepsPerFrame / 2, 1);
      break;
    case SDLK_P: {
      std::string name = "growth_" + std::to_string(ctx->frameCount) + ".bmp";
      ctx->ui.saveScreenshot(name.c_str());
      break;
    }
    default:
      break;
    }
  }
}

static void mainloop(void *arg) {
  context *ctx = static_cast<context *>(arg);
  ctx->frameStart = SDL_GetTicks();

  checkEvents(ctx);

  // show the original skyline for a moment, then let it grow
  if (!ctx->pause) {
    if (ctx->growthFrames >= ctx->growthDelay)
      for (int i = 0; i < ctx->stepsPerFrame; ++i)
        ctx->skyline.step();
    ctx->growthFrames++;
  }

  ctx->ui.clear();
  ctx->skyline.draw(ctx->ui, ctx->debug);
  ctx->ui.present();
  ctx->frameCount++;

#ifndef __EMSCRIPTEN__
  int frameTime = SDL_GetTicks() - ctx->frameStart;
  if (ctx->frameDelay > frameTime)
    SDL_Delay(ctx->frameDelay - frameTime);
#endif
}

int main() {
  static context ctx;
  restart(&ctx);
#ifdef __EMSCRIPTEN__
  emscripten_set_main_loop_arg(mainloop, &ctx, -1, 1);
#else
  while (!ctx.quit)
    mainloop(&ctx);
#endif
  return 0;
}
