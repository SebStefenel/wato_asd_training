#include "planner_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger & logger) : logger_(logger) {}

CellIndex PlannerCore::worldToGrid(
  double wx, double wy,
  const nav_msgs::msg::OccupancyGrid & map) const
{
  int gx = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
  int gy = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(gx, gy);
}

geometry_msgs::msg::PoseStamped PlannerCore::gridToWorld(
  const CellIndex & cell,
  const nav_msgs::msg::OccupancyGrid & map) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id       = "sim_world";
  pose.pose.position.x       = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
  pose.pose.position.y       = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
  pose.pose.orientation.w    = 1.0;
  return pose;
}

double PlannerCore::heuristic(const CellIndex & a, const CellIndex & b) const
{
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool PlannerCore::isTraversable(
  const CellIndex & cell,
  const nav_msgs::msg::OccupancyGrid & map) const
{
  if (cell.x < 0 || cell.x >= static_cast<int>(map.info.width))  return false;
  if (cell.y < 0 || cell.y >= static_cast<int>(map.info.height)) return false;
  // Unknown cells (-1) are treated as traversable so the robot can plan through unexplored space
  int8_t cost = map.data[cell.y * map.info.width + cell.x];
  return cost < OBSTACLE_THRESHOLD;
}

std::vector<CellIndex> PlannerCore::getNeighbors(
  const CellIndex & cell,
  const nav_msgs::msg::OccupancyGrid & map) const
{
  static const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static const int dy[] = {-1, -1, -1, 0, 0,  1, 1, 1};

  std::vector<CellIndex> neighbors;
  neighbors.reserve(8);
  for (int i = 0; i < 8; ++i) {
    CellIndex n(cell.x + dx[i], cell.y + dy[i]);
    if (isTraversable(n, map)) {
      neighbors.push_back(n);
    }
  }
  return neighbors;
}

nav_msgs::msg::Path PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid & map,
  double start_x, double start_y,
  double goal_x,  double goal_y)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "sim_world";

  if (map.data.empty()) {
    RCLCPP_WARN(logger_, "planPath called with empty map");
    return path;
  }

  CellIndex start = worldToGrid(start_x, start_y, map);
  CellIndex goal  = worldToGrid(goal_x,  goal_y,  map);

  // If the robot's cell is in an inflated zone, expand outward to find the
  // nearest free cell and plan from there instead of failing outright.
  if (!isTraversable(start, map)) {
    bool found = false;
    for (int r = 1; r <= 10 && !found; ++r) {
      for (int dy = -r; dy <= r && !found; ++dy) {
        for (int dx = -r; dx <= r && !found; ++dx) {
          if (std::abs(dx) != r && std::abs(dy) != r) continue;  // shell only
          CellIndex candidate(start.x + dx, start.y + dy);
          if (isTraversable(candidate, map)) {
            start = candidate;
            found = true;
          }
        }
      }
    }
    if (!found) {
      RCLCPP_WARN(logger_, "No traversable start cell found near robot position");
      return path;
    }
  }
  if (!isTraversable(goal, map)) {
    bool found = false;
    for (int r = 1; r <= 10 && !found; ++r) {
      for (int dy = -r; dy <= r && !found; ++dy) {
        for (int dx = -r; dx <= r && !found; ++dx) {
          if (std::abs(dx) != r && std::abs(dy) != r) continue;
          CellIndex candidate(goal.x + dx, goal.y + dy);
          if (isTraversable(candidate, map)) {
            goal = candidate;
            found = true;
          }
        }
      }
    }
    if (!found) {
      RCLCPP_WARN(logger_, "No traversable goal cell found near target position");
      return path;
    }
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double,     CellIndexHash> g_score;
  std::unordered_map<CellIndex, bool,       CellIndexHash> closed;

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (closed[current.index]) continue;
    closed[current.index] = true;

    if (current.index == goal) {
      // Reconstruct path from goal back to start, then reverse
      std::vector<geometry_msgs::msg::PoseStamped> poses;
      CellIndex c = goal;
      while (c != start) {
        poses.push_back(gridToWorld(c, map));
        c = came_from[c];
      }
      poses.push_back(gridToWorld(start, map));
      std::reverse(poses.begin(), poses.end());
      path.poses = poses;
      return path;
    }

    for (const auto & neighbor : getNeighbors(current.index, map)) {
      if (closed[neighbor]) continue;

      bool diagonal  = (neighbor.x != current.index.x) && (neighbor.y != current.index.y);
      double move    = diagonal ? 1.414 : 1.0;

      // Strongly penalise cells near obstacles so the planner routes through open space
      int8_t cell_cost = map.data[neighbor.y * map.info.width + neighbor.x];
      double penalty   = (cell_cost > 0) ? (cell_cost / 100.0) * 8.0 : 0.0;

      double tentative_g = g_score[current.index] + move + penalty;

      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
        g_score[neighbor]  = tentative_g;
        came_from[neighbor] = current.index;
        open_set.emplace(neighbor, tentative_g + heuristic(neighbor, goal));
      }
    }
  }

  RCLCPP_WARN(logger_, "No path found from (%.1f, %.1f) to (%.1f, %.1f)",
    start_x, start_y, goal_x, goal_y);
  return path;
}

}  // namespace robot
