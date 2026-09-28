#include "../include/sim_loop.h"
#include <algorithm>
#include <string>

SimLoop::SimLoop() {
  ctx.cableNetwork.generate(ctx.seed, ctx.halfWidth, ctx.halfHeight);
  ctx.cableCar.reset(ctx.cableNetwork, {0, 0});
  ctx.target = ctx.cableCar.getPosition();
}
SimLoop::~SimLoop() {}

void SimLoop::run() {
  ctx.lastTicks = SDL_GetTicks();
#ifdef __EMSCRIPTEN__
  emscripten_set_main_loop_arg(mainloop, &ctx, -1, 1);
#else
  while (!ctx.helperVars.getQuit())
    mainloop(&ctx);
#endif
}

void SimLoop::update(context *ctx) {
  // keep the target on screen
  ctx->target.x = std::clamp(ctx->target.x, -ctx->halfWidth + 1, ctx->halfWidth - 1);
  ctx->target.y = std::clamp(ctx->target.y, -ctx->halfHeight + 1, ctx->halfHeight - 1);
  ctx->cableCar.setTarget(ctx->target);

  // gait planning once per step, physics in substeps
  ctx->cableCar.plan(ctx->cableNetwork, ctx->dt);
  double h = ctx->dt / ctx->substeps;
  for (int i = 0; i < ctx->substeps; ++i) {
    ctx->cableNetwork.clearForces();
    ctx->cableCar.step(ctx->cableNetwork, h);
    ctx->cableNetwork.substep(h);
  }
  ctx->cableCar.updateKinematics(ctx->cableNetwork);

  // store pod position in trajectory
  ctx->helperVars.getTrajectory().push_back(ctx->cableCar.getPosition());

  // fell off the canopy -> respawn
  if (ctx->cableCar.getPosition().y < -ctx->halfHeight - 6)
    ctx->cableCar.reset(ctx->cableNetwork, ctx->target);
}

void SimLoop::render(context *ctx) {
  ctx->ui.clear(); // clears screen

  ctx->cableNetwork.draw(ctx->ui);
  if (ctx->helperVars.getTrajOn())
    ctx->ui.drawTrajectory(ctx->helperVars.getTrajectory(), 200);
  ctx->cableCar.draw(ctx->ui, ctx->helperVars.getDebug());

  // target crosshair
  Vec2 t = ctx->target;
  ctx->ui.setAlpha(200);
  ctx->ui.drawCircle(t, 0.4);
  ctx->ui.drawLine(t - Vec2(0.7, 0), t - Vec2(0.2, 0));
  ctx->ui.drawLine(t + Vec2(0.2, 0), t + Vec2(0.7, 0));
  ctx->ui.drawLine(t - Vec2(0, 0.7), t - Vec2(0, 0.2));
  ctx->ui.drawLine(t + Vec2(0, 0.2), t + Vec2(0, 0.7));
  // controller switched off: cross through the pod
  if (!ctx->cableCar.getControllerState()) {
    Vec2 p = ctx->cableCar.getPosition();
    ctx->ui.setAlpha(255);
    ctx->ui.drawThickLine(p - Vec2(0.6, 0.6), p + Vec2(0.6, 0.6), 3);
    ctx->ui.drawThickLine(p - Vec2(0.6, -0.6), p + Vec2(0.6, -0.6), 3);
  }

  ctx->ui.present(); // shows rendered objects
}

void SimLoop::mainloop(void *arg) {
  context *ctx = static_cast<context *>(arg);

  ctx->frameStart = SDL_GetTicks(); // limit framerate (see end of loop)

  // check for Keyboard inputs
  ctx->eventChecks.checkEvents(ctx->helperVars, ctx->cableCar,
                               ctx->cableNetwork, ctx->ui, ctx->target);

  // respawn the pod / grow new cables
  if (ctx->helperVars.getReset()) {
    ctx->cableCar.reset(ctx->cableNetwork, ctx->target);
    ctx->helperVars.getTrajectory().clear();
    ctx->helperVars.toggleReset();
  }
  if (ctx->helperVars.getRegenerate()) {
    ctx->seed++;
    ctx->cableNetwork.generate(ctx->seed, ctx->halfWidth, ctx->halfHeight);
    ctx->cableCar.reset(ctx->cableNetwork, ctx->target);
    ctx->helperVars.getTrajectory().clear();
    ctx->helperVars.toggleRegenerate();
  }

  // fixed time steps, independent of the display refresh rate
  double elapsed = (ctx->frameStart - ctx->lastTicks) / 1000.0;
  ctx->lastTicks = ctx->frameStart;
  if (ctx->helperVars.getPause()) {
    ctx->accumulator = 0;
  } else {
    ctx->eventChecks.moveTarget(ctx->target, elapsed);
    ctx->accumulator += std::min(elapsed, 0.1);
    while (ctx->accumulator >= ctx->dt) {
      update(ctx);
      ctx->accumulator -= ctx->dt;
    }
  }

  render(ctx);

  if (ctx->helperVars.getScreenshot()) {
    std::string name = "screenshot_" + std::to_string(ctx->frameCount) + ".bmp";
    ctx->ui.saveScreenshot(name.c_str());
    ctx->helperVars.toggleScreenshot();
  }

  ctx->frameCount += 1; // count Frame

#ifndef __EMSCRIPTEN__
  // frame time to limit FPS
  int frameTime = SDL_GetTicks() - ctx->frameStart;
  if (ctx->frameDelay > frameTime) {
    SDL_Delay(ctx->frameDelay - frameTime);
  }
#endif
}
