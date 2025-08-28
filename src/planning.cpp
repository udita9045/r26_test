#include "planning.h"
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>

using namespace std;

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = grid.size();
  cols = grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
  vector<pair<int, int>> path;

  // Min-heap priority queue for A* (fScore, (x,y))
  using Node = pair<double, pair<int, int>>;
  priority_queue<Node, vector<Node>, greater<Node>> open;

  // Maps for gScore (cost so far) and parent tracking
  unordered_map<int, double> gScore;
  unordered_map<int, pair<int,int>> cameFrom;

  auto index = [&](int x, int y) { return x * cols + y; };

  // Initialize
  gScore[index(start.first, start.second)] = 0.0;
  open.push({heuristic(start.first, start.second, goal.first, goal.second), start});

  // Directions (4-connected grid: up, down, left, right)
  vector<pair<int,int>> dirs = {{1,0}, {-1,0}, {0,1}, {0,-1}};

  while (!open.empty()) {
    auto current = open.top().second;
    open.pop();

    int cx = current.first;
    int cy = current.second;

    if (current == goal) {
      // reconstruct path
      pair<int,int> node = goal;
      while (cameFrom.find(index(node.first, node.second)) != cameFrom.end()) {
        path.push_back(node);
        node = cameFrom[index(node.first, node.second)];
      }
      path.push_back(start);
      reverse(path.begin(), path.end());
      return path;
    }

    for (auto &d : dirs) {
      int nx = cx + d.first;
      int ny = cy + d.second;

      if (!isvalid(nx, ny)) continue;

      double tentative_g = gScore[index(cx, cy)] + 1.0; // cost = 1 per step

      if (!gScore.count(index(nx, ny)) || tentative_g < gScore[index(nx, ny)]) {
        cameFrom[index(nx, ny)] = {cx, cy};
        gScore[index(nx, ny)] = tentative_g;
        double fScore = tentative_g + heuristic(nx, ny, goal.first, goal.second);
        open.push({fScore, {nx, ny}});
      }
    }
  }
  return path;
}
