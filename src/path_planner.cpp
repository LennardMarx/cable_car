#include "../include/path_planner.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

PathPlanner::PathPlanner() : support(cols * rows, 0) {}
PathPlanner::~PathPlanner() {}

void PathPlanner::setExtents(double _halfWidth, double _halfHeight) {
  halfWidth = _halfWidth;
  halfHeight = _halfHeight;
  cols = std::max(1, (int)std::ceil(2 * halfWidth / cellSize));
  rows = std::max(1, (int)std::ceil(2 * halfHeight / cellSize));
  support.assign(cols * rows, 0);
  path.clear();
  progress = 0;
}

bool PathPlanner::inGrid(int _c, int _r) const {
  return _c >= 0 && _c < cols && _r >= 0 && _r < rows;
}

int PathPlanner::cellIndex(int _c, int _r) const { return _r * cols + _c; }

int PathPlanner::cellAt(const Vec2 &_p) const {
  int c = std::clamp((int)((_p.x + halfWidth) / cellSize), 0, cols - 1);
  int r = std::clamp((int)((_p.y + halfHeight) / cellSize), 0, rows - 1);
  return cellIndex(c, r);
}

Vec2 PathPlanner::cellCenter(int _i) const {
  return {-halfWidth + (_i % cols + 0.5) * cellSize,
          -halfHeight + (_i / cols + 0.5) * cellSize};
}

bool PathPlanner::walkable(int _i) const { return support[_i] >= minSupport; }

// count for every cell the distinct vines a pod there could grab
void PathPlanner::buildSupport(const CableNetwork &_net) {
  std::fill(support.begin(), support.end(), 0);
  std::vector<int> lastCable(cols * rows, -1);
  const auto &cables = _net.getCables();
  int span = std::ceil(reach / cellSize);

  for (int c = 0; c < (int)cables.size(); ++c) {
    int maxU = cables[c].ids.size() - 1;
    for (double u = 0; u <= maxU; u += 1) {
      Vec2 p = _net.getPoint({c, u});
      int pc = (p.x + halfWidth) / cellSize;
      int pr = (p.y + halfHeight) / cellSize;
      for (int r = pr - span; r <= pr + span; ++r) {
        for (int col = pc - span; col <= pc + span; ++col) {
          if (!inGrid(col, r))
            continue;
          int i = cellIndex(col, r);
          if (lastCable[i] == c || (cellCenter(i) - p).length() > reach)
            continue;
          lastCable[i] = c; // count every vine only once per cell
          support[i]++;
        }
      }
    }
  }
}

// closest cell where the pod can hold on
int PathPlanner::nearestWalkable(const Vec2 &_p) const {
  int best = -1;
  double bestDist = std::numeric_limits<double>::infinity();
  for (int i = 0; i < cols * rows; ++i) {
    if (!walkable(i))
      continue;
    double d = (cellCenter(i) - _p).length();
    if (d < bestDist) {
      bestDist = d;
      best = i;
    }
  }
  return best;
}

// straight line only through walkable cells
bool PathPlanner::lineOfSight(const Vec2 &_a, const Vec2 &_b) const {
  double len = (_b - _a).length();
  int steps = std::max(1, (int)(len / (cellSize * 0.25)));
  for (int k = 0; k <= steps; ++k)
    if (!walkable(cellAt(lerp(_a, _b, (double)k / steps))))
      return false;
  return true;
}

// string pulling: skip waypoints which can be seen directly
void PathPlanner::smoothPath(std::vector<Vec2> &_path) const {
  if (_path.size() < 3)
    return;
  std::vector<Vec2> smooth = {_path.front()};
  size_t i = 0;
  while (i + 1 < _path.size()) {
    size_t j = _path.size() - 1;
    while (j > i + 1 && !lineOfSight(_path[i], _path[j]))
      j--;
    smooth.push_back(_path[j]);
    i = j;
  }
  _path = smooth;
}

void PathPlanner::plan(const CableNetwork &_net, const Vec2 &_start,
                       const Vec2 &_target) {
  buildSupport(_net);
  path.clear();
  progress = 0;

  int startCell = walkable(cellAt(_start)) ? cellAt(_start)
                                           : nearestWalkable(_start);
  if (startCell < 0) {
    goal = _target;
    path = {_start, _target};
    return;
  }

  // dijkstra from the pod over all cells it can hold on in,
  // steps through weakly supported cells cost more
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> cost(cols * rows, inf);
  std::vector<int> parent(cols * rows, -1);
  using Node = std::pair<double, int>; // (cost, cell)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  cost[startCell] = 0;
  open.push({0, startCell});

  while (!open.empty()) {
    auto [d, i] = open.top();
    open.pop();
    if (d > cost[i])
      continue; // outdated entry
    int c = i % cols, r = i / cols;
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if ((dr == 0 && dc == 0) || !inGrid(c + dc, r + dr))
          continue;
        int n = cellIndex(c + dc, r + dr);
        if (!walkable(n))
          continue;
        double step = std::sqrt((double)(dr * dr + dc * dc)) * cellSize;
        double newCost = cost[i] + step * (1 + weakPenalty / support[n]);
        if (newCost < cost[n]) {
          cost[n] = newCost;
          parent[n] = i;
          open.push({newCost, n});
        }
      }
    }
  }

  // goal: the reachable cell closest to the target (the target itself if
  // it is supported), the last bit to the target is taken directly
  int goalCell = cellAt(_target);
  double bestDist = cost[goalCell] < inf ? 0 : inf;
  for (int i = 0; i < cols * rows && bestDist > 0; ++i) {
    if (cost[i] == inf)
      continue;
    double d = (cellCenter(i) - _target).length();
    if (d < bestDist - 1e-9) {
      bestDist = d;
      goalCell = i;
    }
  }
  goal = goalCell == cellAt(_target) ? _target : cellCenter(goalCell);

  // already on the last bit beyond the supported area -> keep going
  if ((_target - _start).length() < (_target - goal).length()) {
    path = {_start, _target};
    return;
  }

  for (int i = goalCell; i >= 0; i = parent[i])
    path.push_back(cellCenter(i));
  std::reverse(path.begin(), path.end());
  path.front() = _start;
  path.back() = goal;
  smoothPath(path);
  if (goal.x != _target.x || goal.y != _target.y)
    path.push_back(_target);
}

// carrot: the point one lookahead distance further along the path than the
// pod's closest point on it
Vec2 PathPlanner::getWaypoint(const Vec2 &_pos) {
  if (path.empty())
    return _pos;
  if (path.size() == 1)
    return path.front();

  // advance along the path while the pod is closer to the next segment
  auto distToSegment = [&](size_t _k, double &_t) {
    Vec2 a = path[_k], b = path[_k + 1];
    Vec2 ab = b - a;
    double l2 = ab.dot(ab);
    _t = l2 > 1e-9 ? std::clamp((_pos - a).dot(ab) / l2, 0.0, 1.0) : 0;
    return (lerp(a, b, _t) - _pos).length();
  };
  double t;
  double d = distToSegment(progress, t);
  while (progress + 2 < path.size()) {
    double tNext;
    double dNext = distToSegment(progress + 1, tNext);
    if (dNext > d && t < 1)
      break;
    progress++;
    d = dNext;
    t = tNext;
  }

  // walk the lookahead distance along the path
  Vec2 p = lerp(path[progress], path[progress + 1], t);
  double left = lookahead;
  for (size_t k = progress; k + 1 < path.size(); ++k) {
    Vec2 end = path[k + 1];
    double segLeft = (end - p).length();
    if (segLeft >= left)
      return p + (end - p).normalized() * left;
    left -= segLeft;
    p = end;
  }
  return path.back();
}

void PathPlanner::draw(UI &_ui, bool _debug) {
  // cells where the pod can hold on
  if (_debug) {
    for (int i = 0; i < cols * rows; ++i) {
      if (!walkable(i))
        continue;
      _ui.setAlpha(std::min(12 * support[i], 60));
      Vec2 c = cellCenter(i);
      double h = cellSize * 0.4;
      _ui.fillPolygon({c + Vec2(-h, -h), c + Vec2(h, -h), c + Vec2(h, h),
                       c + Vec2(-h, h)});
    }
  }

  // planned path
  _ui.setAlpha(90);
  for (size_t k = progress; k + 1 < path.size(); ++k)
    _ui.drawLine(path[k], path[k + 1]);
  for (size_t k = progress + 1; k < path.size(); ++k)
    _ui.fillCircle(path[k], 2.5 / _ui.scale);
}
