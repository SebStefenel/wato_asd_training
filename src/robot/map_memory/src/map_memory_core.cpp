#include "map_memory_core.hpp"
#include <cmath>

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger & logger) : logger_(logger)
{
  global_map_.header.frame_id        = "sim_world";
  global_map_.info.resolution        = RESOLUTION;
  global_map_.info.width             = WIDTH;
  global_map_.info.height            = HEIGHT;
  global_map_.info.origin.position.x = ORIGIN_X;
  global_map_.info.origin.position.y = ORIGIN_Y;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(WIDTH * HEIGHT, -1);  // -1 = unknown
}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid & costmap)
{
  latest_costmap_   = costmap;
  costmap_received_ = true;
}

void MapMemoryCore::updateRobotPose(double x, double y, double yaw)
{
  robot_x_   = x;
  robot_y_   = y;
  robot_yaw_ = yaw;

  double dx = x - last_update_x_;
  double dy = y - last_update_y_;
  if (std::sqrt(dx * dx + dy * dy) >= UPDATE_DISTANCE) {
    should_update_ = true;
  }
}

bool MapMemoryCore::shouldUpdateMap() const
{
  return should_update_ && costmap_received_;
}

void MapMemoryCore::integrateCostmap()
{
  should_update_ = false;
  last_update_x_ = robot_x_;
  last_update_y_ = robot_y_;

  const auto & cm = latest_costmap_;
  double cos_yaw  = std::cos(robot_yaw_);
  double sin_yaw  = std::sin(robot_yaw_);

  for (int cy = 0; cy < static_cast<int>(cm.info.height); ++cy) {
    for (int cx = 0; cx < static_cast<int>(cm.info.width); ++cx) {
      int8_t val = cm.data[cy * cm.info.width + cx];
      if (val < 0) continue;

      // Cell centre in lidar (local) frame
      double lx = cm.info.origin.position.x + (cx + 0.5) * cm.info.resolution;
      double ly = cm.info.origin.position.y + (cy + 0.5) * cm.info.resolution;

      // Rotate + translate into world frame.
      // odometry_spoof tracks robot/chassis/lidar so the odom position IS the lidar position.
      double wx = robot_x_ + lx * cos_yaw - ly * sin_yaw;
      double wy = robot_y_ + lx * sin_yaw + ly * cos_yaw;

      int gx = static_cast<int>((wx - ORIGIN_X) / RESOLUTION);
      int gy = static_cast<int>((wy - ORIGIN_Y) / RESOLUTION);

      if (gx < 0 || gx >= WIDTH || gy < 0 || gy >= HEIGHT) continue;

      int idx        = gy * WIDTH + gx;
      int8_t old_val = global_map_.data[idx];
      if (old_val < 0 || val > old_val) {
        global_map_.data[idx] = val;
      }
    }
  }
}

const nav_msgs::msg::OccupancyGrid & MapMemoryCore::getGlobalMap() const
{
  return global_map_;
}

}  // namespace robot
