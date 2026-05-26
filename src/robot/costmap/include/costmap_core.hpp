#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>

namespace robot {

class CostmapCore {
public:
  explicit CostmapCore(const rclcpp::Logger & logger);

  nav_msgs::msg::OccupancyGrid processLaserScan(const sensor_msgs::msg::LaserScan & scan);

private:
  void initGrid();
  void markObstacle(int gx, int gy);
  void inflateObstacles();

  rclcpp::Logger logger_;

  static constexpr double RESOLUTION       = 0.1;   // m/cell
  static constexpr int    WIDTH            = 400;   // cells (40 m total)
  static constexpr int    HEIGHT           = 400;
  static constexpr double INFLATION_RADIUS = 2.0;   // m
  static constexpr int8_t MAX_COST         = 100;

  std::vector<int8_t> grid_;
};

}  // namespace robot

#endif
