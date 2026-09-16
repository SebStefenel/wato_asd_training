# Autonomous Navigation & Obstacle Avoidance Stack (ROS 2 Humble)

## What this does

I built the "brain" for a simulated robot that can drive itself to any point you give it without hitting anything. It works in four steps:

1. **See:** uses a laser sensor (LiDAR) to detect nearby walls and obstacles.
2. **Remember:** combines those snapshots into a map that grows as the robot explores.
3. **Plan:** finds the shortest safe route to the goal while keeping a buffer from obstacles.
4. **Drive:** steers the robot along that route and slows down for sharp turns.

As the robot moves, it keeps updating its map and recalculating its route, so it can work around obstacles it didn't know about when it started.

**What I built:** the four navigation nodes in `src/robot/` (`costmap`, `map_memory`, `planner`, `control`), written in C++, plus the Foxglove visualization setup. The Gazebo simulation, Docker/`watod` tooling, and the `odometry_spoof` node were provided as starter code by WATonomous.

---

## Overview

This is a modular 2D navigation system written in C++ on **ROS 2 Humble**, containerized with Docker, and simulated in Gazebo. A differential-drive robot with a 2D LiDAR builds a map of its surroundings in real time, plans collision-free paths with A\*, and follows them with a Pure Pursuit controller.

## Architecture

The system is split into four independent ROS 2 nodes that communicate over standard topics. Robot position comes from `/odom/filtered`, which the provided `odometry_spoof` node publishes from the simulator.

```
               +-------------------+
/lidar ------> |   Costmap Node    |
               +---------+---------+
                         | /costmap (OccupancyGrid)
                         v
               +-------------------+
/odom/filtered>|  Map Memory Node  |
               +---------+---------+
                         | /map (Global OccupancyGrid)
                         v
               +-------------------+
/goal_point -> |   Planner Node    | <--- /odom/filtered
               +---------+---------+
                         | /path (nav_msgs/Path)
                         v
               +-------------------+
               |   Control Node    | <--- /odom/filtered
               +---------+---------+
                         | /cmd_vel (geometry_msgs/Twist)
                         v
                  [Robot Base]
```

### Topic Interfaces

| Node             | Subscribes to                           | Publishes  | Message Types                                       |
| ---------------- | --------------------------------------- | ---------- | --------------------------------------------------- |
| **`costmap`**    | `/lidar`                                | `/costmap` | `sensor_msgs/LaserScan` -> `nav_msgs/OccupancyGrid` |
| **`map_memory`** | `/costmap`, `/odom/filtered`            | `/map`     | `nav_msgs/OccupancyGrid`, `nav_msgs/Odometry`       |
| **`planner`**    | `/map`, `/odom/filtered`, `/goal_point` | `/path`    | `geometry_msgs/PointStamped` -> `nav_msgs/Path`     |
| **`control`**    | `/path`, `/odom/filtered`               | `/cmd_vel` | `geometry_msgs/Twist`                               |

---

## How Each Node Works

### 1. Costmap Node (`src/robot/costmap`)

Turns each LiDAR scan into a local map of nearby obstacles.

- **Scan to grid:** converts each range and angle reading into $(x, y)$ coordinates in the LiDAR frame, skipping invalid readings (NaN, infinite, or outside the sensor's range).
- **Local grid:** a 40 m × 40 m grid at 0.1 m per cell, centred on the LiDAR.
- **Obstacle inflation:** spreads cost outward from each obstacle within a 2 m radius, fading linearly with distance:
  `cost = max_cost * (1 - d / inflation_radius)`
  This creates a buffer that pushes planned paths away from walls.

### 2. Map Memory Node (`src/robot/map_memory`)

Stitches local costmaps into one global map of the 30 m × 30 m world.

- **Frame transform:** rotates and shifts each local costmap cell into the world frame using the robot's current position and heading.
- **Distance-based updates:** only merges a new costmap after the robot has moved at least 1.5 m, which avoids redundant work.
- **Max-cost merge:** fills in cells that were unknown and keeps the higher cost wherever old and new data overlap. Obstacles are never erased once mapped, which suits this static environment.
- **Publishing:** sends the global map once per second.

### 3. Planner Node (`src/robot/planner`)

Finds a safe route from the robot to the goal.

- **Two states:**
  - `WAITING_FOR_GOAL`: idle until a target arrives on `/goal_point`.
  - `NAVIGATING`: plans a path and checks every 0.5 s whether the robot is within 0.5 m of the goal. Once it is, the node publishes an empty path and returns to waiting.
- **A\* search:** searches the global map in 8 directions using a Euclidean distance heuristic.
  - Only cells at full obstacle cost are blocked.
  - Cells in the inflation zone are passable but penalized in proportion to their cost, so the planner prefers open space.
  - Unknown cells are treated as passable so the robot can plan through areas it hasn't explored yet.
- **Start/goal recovery:** if the robot or goal sits on a blocked cell, the planner searches outward (up to 10 cells) for the nearest free cell and plans from there.
- **Replanning:** recalculates the path on every map update and whenever a new goal arrives. If replanning fails, it keeps the previous path so the robot doesn't stall at tight corners.

### 4. Control Node (`src/robot/control`)

Steers the robot along the planned path, running at 10 Hz.

- **Target selection:** finds the closest point on the path, then looks ahead from there to the first point at least 1 m away. This keeps the robot from tracking points it has already passed.
- **Pure Pursuit steering:** with $\alpha$ as the angle to the target point in the robot's frame and $L$ as the distance to it:
  $$\kappa = \frac{2 \sin(\alpha)}{L}, \quad \omega_z = v_{\text{base}} \cdot \kappa$$
  where $v_{\text{base}} = 0.4$ m/s.
- **Turn slowdown:** forward speed scales down with how sharp the turn is, to a minimum of 30%:
  $$v_x = v_{\text{base}} \cdot \max\left(0.3,\ 1 - \frac{|\alpha|}{\pi}\right)$$
  This keeps the robot's body from swinging into corners.
- **Stopping:** outputs zero velocity when there is no path or the robot is within 0.5 m of the path's end.

### Tuning

Tuning values such as grid resolution, inflation radius, update distance, lookahead distance, speed, and goal tolerance are currently constants in each node's header file (`include/*_core.hpp`, `include/planner_node.hpp`). The `config/params.yaml` files are in place for moving these to ROS parameters later.

---

## Running It

With Docker installed, from the repository root:

```bash
./watod up      # build and start the robot, Gazebo, and Foxglove containers
./watod down    # stop and remove the containers
```

The active modules (`robot gazebo vis_tools`) are set in `watod-config.sh`. Connect Foxglove Studio to the Foxglove bridge to view the map, path, and robot, and use a published point on `/goal_point` to send the robot a destination.

---

## Tech Stack

- **Language:** C++ (`rclcpp`)
- **Robotics middleware:** ROS 2 Humble
- **Simulation & visualization:** Gazebo (Ignition), Foxglove Studio
- **Containerization:** Docker, Docker Compose
- **Build system:** CMake, `ament_cmake`, Colcon
- **Supported platforms:** Linux (Ubuntu 22.04+), Windows via WSL2, macOS

---

## Repository Structure

```text
wato_asd_training/
├── config/                 # Foxglove layout
├── docker/                 # Dockerfiles (provided; Foxglove setup added)
├── modules/                # Docker Compose files for each module (provided)
├── src/
│   ├── robot/
│   │   ├── costmap/        # LiDAR costmap + obstacle inflation      (mine)
│   │   ├── map_memory/     # Global map stitching                    (mine)
│   │   ├── planner/        # A* planner + goal state machine         (mine)
│   │   ├── control/        # Pure Pursuit controller                 (mine)
│   │   ├── odometry_spoof/ # Publishes robot pose from the simulator (provided)
│   │   └── bringup_robot/  # Launch file for all robot nodes         (provided)
│   └── gazebo/             # Simulated world and robot               (provided)
├── watod                   # Container orchestration script          (provided)
├── watod-config.sh         # Active module configuration
└── README.md
```
