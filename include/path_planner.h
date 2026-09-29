#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#pragma once
#include "UI.h"
#include "cable_network.h"
#include "vec2.h"
#include <vector>

// A* on a grid of pod positions, rated by how many vines are in reach
class PathPlanner {
public:
  PathPlanner();
  ~PathPlanner();

  void setExtents(double, double); // visible world [m]
  void plan(const CableNetwork &, const Vec2 &, const Vec2 &);
  Vec2 getWaypoint(const Vec2 &); // point on the path ahead of the pod
  Vec2 getGoal() const { return goal; }
  void draw(UI &, bool);

private:
  void buildSupport(const CableNetwork &);
  bool inGrid(int, int) const;
  int cellIndex(int, int) const;
  int cellAt(const Vec2 &) const;
  Vec2 cellCenter(int) const;
  bool walkable(int) const;
  int nearestWalkable(const Vec2 &) const;
  bool lineOfSight(const Vec2 &, const Vec2 &) const;
  void smoothPath(std::vector<Vec2> &) const;

  // grid over the visible world
  const double cellSize = 1.0;
  double halfWidth = 30, halfHeight = 20;
  int cols = 2 * halfWidth / cellSize;
  int rows = 2 * halfHeight / cellSize;

  std::vector<int> support; // distinct vines within reach per cell
  std::vector<Vec2> path;   // smoothed waypoints from start to goal
  size_t progress = 0;      // current segment of the path
  Vec2 goal;

  const double reach = 4.5;    // pod center to vine (arm reach + shoulder)
  const int minSupport = 3;    // vines needed to hold on safely
  const double weakPenalty = 8; // extra cost in cells with few vines
  const double lookahead = 3;  // distance of the waypoint ahead of the pod
};

#endif
