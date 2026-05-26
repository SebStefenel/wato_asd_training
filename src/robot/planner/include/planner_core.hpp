#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <queue>
#include <unordered_map>

namespace robot {

// ── Supporting structures (provided by assignment) ──────────────────────────

struct CellIndex {
  int x, y;
  CellIndex(int xx = 0, int yy = 0) : x(xx), y(yy) {}
  bool operator==(const CellIndex & o) const { return x == o.x && y == o.y; }
  bool operator!=(const CellIndex & o) const { return !(*this == o); }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex & idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode & a, const AStarNode & b) {
    return a.f_score > b.f_score;  // min-heap
  }
};

// ── Planner ──────────────────────────────────────────────────────────────────

class PlannerCore {
public:
  explicit PlannerCore(const rclcpp::Logger & logger);

  nav_msgs::msg::Path planPath(
    const nav_msgs::msg::OccupancyGrid & map,
    double start_x, double start_y,
    double goal_x,  double goal_y);

private:
  CellIndex worldToGrid(
    double wx, double wy,
    const nav_msgs::msg::OccupancyGrid & map) const;

  geometry_msgs::msg::PoseStamped gridToWorld(
    const CellIndex & cell,
    const nav_msgs::msg::OccupancyGrid & map) const;

  double heuristic(const CellIndex & a, const CellIndex & b) const;

  bool isTraversable(
    const CellIndex & cell,
    const nav_msgs::msg::OccupancyGrid & map) const;

  std::vector<CellIndex> getNeighbors(
    const CellIndex & cell,
    const nav_msgs::msg::OccupancyGrid & map) const;

  rclcpp::Logger logger_;

  // Cells with cost >= this threshold are treated as obstacles
  // Only block cells on actual obstacle surfaces (cost==100); inflation zone is costly but passable
  static constexpr int OBSTACLE_THRESHOLD = 99;
};

}  // namespace robot

#endif
