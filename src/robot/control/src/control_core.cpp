#include "control_core.hpp"
#include <cmath>
#include <limits>

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger & logger) : logger_(logger) {}

double ControlCore::computeDistance(double x1, double y1, double x2, double y2) const
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

int ControlCore::findLookaheadIndex(
  const nav_msgs::msg::Path & path, double rx, double ry) const
{
  // First find the closest waypoint to avoid tracking already-passed waypoints
  int closest  = 0;
  double min_d = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(path.poses.size()); ++i) {
    double d = computeDistance(rx, ry,
      path.poses[i].pose.position.x,
      path.poses[i].pose.position.y);
    if (d < min_d) { min_d = d; closest = i; }
  }

  // From the closest point, find the first one at or beyond lookahead distance
  for (int i = closest; i < static_cast<int>(path.poses.size()); ++i) {
    double d = computeDistance(rx, ry,
      path.poses[i].pose.position.x,
      path.poses[i].pose.position.y);
    if (d >= LOOKAHEAD_DISTANCE) return i;
  }

  return static_cast<int>(path.poses.size()) - 1;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const nav_msgs::msg::Path & path,
  double robot_x, double robot_y, double robot_yaw)
{
  geometry_msgs::msg::Twist cmd;  // zero-initialised

  if (path.poses.empty()) return cmd;

  // Stop if we've reached the end of the path
  const auto & last = path.poses.back().pose.position;
  if (computeDistance(robot_x, robot_y, last.x, last.y) < GOAL_TOLERANCE) {
    return cmd;
  }

  int idx        = findLookaheadIndex(path, robot_x, robot_y);
  double tgt_x   = path.poses[idx].pose.position.x;
  double tgt_y   = path.poses[idx].pose.position.y;

  double dx      = tgt_x - robot_x;
  double dy      = tgt_y - robot_y;
  double L       = std::sqrt(dx * dx + dy * dy);
  if (L < 1e-6) return cmd;

  // α = angle to target in robot frame
  double alpha   = std::atan2(dy, dx) - robot_yaw;
  while (alpha >  M_PI) alpha -= 2.0 * M_PI;
  while (alpha < -M_PI) alpha += 2.0 * M_PI;

  // Pure Pursuit: κ = 2·sin(α) / L
  // Slow down proportional to turn sharpness so the body doesn't sweep into corners
  double curvature   = 2.0 * std::sin(alpha) / L;
  double attenuation = std::max(0.3, 1.0 - std::abs(alpha) / M_PI);
  cmd.linear.x       = LINEAR_SPEED * attenuation;
  cmd.angular.z      = LINEAR_SPEED * curvature;

  return cmd;
}

}  // namespace robot
