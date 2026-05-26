#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot {

class ControlCore {
public:
  explicit ControlCore(const rclcpp::Logger & logger);

  geometry_msgs::msg::Twist computeVelocity(
    const nav_msgs::msg::Path & path,
    double robot_x, double robot_y, double robot_yaw);

private:
  int    findLookaheadIndex(const nav_msgs::msg::Path & path, double rx, double ry) const;
  double computeDistance(double x1, double y1, double x2, double y2) const;

  rclcpp::Logger logger_;

  static constexpr double LOOKAHEAD_DISTANCE = 1.0;  // m
  static constexpr double LINEAR_SPEED       = 0.4;  // m/s
  static constexpr double GOAL_TOLERANCE     = 0.5;  // m
};

}  // namespace robot

#endif
