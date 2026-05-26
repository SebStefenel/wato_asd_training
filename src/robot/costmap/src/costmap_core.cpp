#include "costmap_core.hpp"
#include <cmath>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger & logger) : logger_(logger) {}

void CostmapCore::initGrid()
{
  grid_.assign(WIDTH * HEIGHT, 0);
}

void CostmapCore::markObstacle(int gx, int gy)
{
  if (gx < 0 || gx >= WIDTH || gy < 0 || gy >= HEIGHT) return;
  grid_[gy * WIDTH + gx] = MAX_COST;
}

void CostmapCore::inflateObstacles()
{
  std::vector<int8_t> inflated = grid_;
  int inflation_cells = static_cast<int>(INFLATION_RADIUS / RESOLUTION);

  for (int y = 0; y < HEIGHT; ++y) {
    for (int x = 0; x < WIDTH; ++x) {
      if (grid_[y * WIDTH + x] != MAX_COST) continue;

      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          int nx = x + dx, ny = y + dy;
          if (nx < 0 || nx >= WIDTH || ny < 0 || ny >= HEIGHT) continue;

          double dist = std::sqrt(dx * dx + dy * dy) * RESOLUTION;
          if (dist > INFLATION_RADIUS) continue;

          int8_t cost = static_cast<int8_t>(MAX_COST * (1.0 - dist / INFLATION_RADIUS));
          if (cost > inflated[ny * WIDTH + nx]) {
            inflated[ny * WIDTH + nx] = cost;
          }
        }
      }
    }
  }
  grid_ = inflated;
}

nav_msgs::msg::OccupancyGrid CostmapCore::processLaserScan(
  const sensor_msgs::msg::LaserScan & scan)
{
  initGrid();

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    if (range < scan.range_min || range > scan.range_max) continue;
    if (std::isnan(range) || std::isinf(range)) continue;

    double angle = scan.angle_min + i * scan.angle_increment;
    double lx = range * std::cos(angle);
    double ly = range * std::sin(angle);

    // Grid origin is at the bottom-left corner, which is (-20, -20) relative to lidar
    int gx = static_cast<int>((lx + WIDTH  * RESOLUTION / 2.0) / RESOLUTION);
    int gy = static_cast<int>((ly + HEIGHT * RESOLUTION / 2.0) / RESOLUTION);
    markObstacle(gx, gy);
  }

  inflateObstacles();

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id        = "robot/chassis/lidar";
  msg.info.resolution        = RESOLUTION;
  msg.info.width             = WIDTH;
  msg.info.height            = HEIGHT;
  msg.info.origin.position.x = -(WIDTH  * RESOLUTION / 2.0);
  msg.info.origin.position.y = -(HEIGHT * RESOLUTION / 2.0);
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data = grid_;

  return msg;
}

}  // namespace robot
