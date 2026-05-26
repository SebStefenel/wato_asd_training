#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot {

class MapMemoryCore {
public:
  explicit MapMemoryCore(const rclcpp::Logger & logger);

  void updateCostmap(const nav_msgs::msg::OccupancyGrid & costmap);
  void updateRobotPose(double x, double y, double yaw);
  bool shouldUpdateMap() const;
  void integrateCostmap();
  const nav_msgs::msg::OccupancyGrid & getGlobalMap() const;

private:
  rclcpp::Logger logger_;

  // 30 m × 30 m world, walls at ±15 m
  static constexpr double RESOLUTION       = 0.1;
  static constexpr int    WIDTH            = 300;
  static constexpr int    HEIGHT           = 300;
  static constexpr double ORIGIN_X         = -15.0;
  static constexpr double ORIGIN_Y         = -15.0;
  static constexpr double UPDATE_DISTANCE  = 1.5;  // m between map fusions

  nav_msgs::msg::OccupancyGrid global_map_;
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  bool costmap_received_ = false;
  bool should_update_    = false;

  double robot_x_        = 0.0;
  double robot_y_        = 0.0;
  double robot_yaw_      = 0.0;
  double last_update_x_  = 1e9;  // force first update
  double last_update_y_  = 1e9;
};

}  // namespace robot

#endif
