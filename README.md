# Autonomous Navigation & Obstacle Avoidance Stack (ROS 2 Humble)

A modular, 2D autonomous navigation system developed in C++ using **ROS 2 Humble**, containerized via Docker, and simulated in Gazebo. The stack enables a differential-drive robot equipped with a 2D LiDAR scanner and wheel odometry to map its environment in real-time, plan collision-free paths, and track trajectories toward arbitrary goal points while dynamically avoiding static obstacles.

---

## Architecture Overview

The system decomposes autonomous navigation into four decoupled ROS 2 nodes communicating over standard topics:

```text
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

### Dataflow & Topic Interfaces

| Node | Subscribed Topics | Published Topics | Key Message Types |
| :--- | :--- | :--- | :--- |
| **`costmap`** | `/lidar` | `/costmap` | `sensor_msgs/LaserScan` -> `nav_msgs/OccupancyGrid` |
| **`map_memory`** | `/costmap`, `/odom/filtered` | `/map` | `nav_msgs/OccupancyGrid`, `nav_msgs/Odometry` |
| **`planner`** | `/map`, `/odom/filtered`, `/goal_point` | `/path` | `geometry_msgs/PointStamped` -> `nav_msgs/Path` |
| **`control`** | `/path`, `/odom/filtered` | `/cmd_vel` | `geometry_msgs/Twist` |

---

## Pipeline Breakdown

### 1. Costmap Node (`src/robot/costmap`)
* **Polar-to-Cartesian Projection:** Converts raw 2D `sensor_msgs/msg/LaserScan` range and bearing arrays into Cartesian $(x, y)$ coordinates in the robot frame.
* **Discretization & Grid Mapping:** Discretizes local space into a 2D occupancy grid ($0.1\text{ m/cell}$) centered around the vehicle base.
* **Obstacle Inflation:** Applies Euclidean distance-based inflation around detected obstacle points using a linear decay model:
  `cost = max_cost * (1 - d / r_inflation)`
  This provides a continuous potential field buffer to keep path planning clear of robot footprint collisions.

### 2. Map Memory Node (`src/robot/map_memory`)
* **Global Frame Stitched Mapping:** Accumulates local costmaps over time into a unified global map coordinate frame.
* **Distance-Thresholded Integration:** Monitors cumulative robot displacement via `/odom/filtered` updates; triggers global map fusion only when displacement exceeds a configured threshold to reduce unnecessary computation.
* **Linear Fusion & State Retention:** Fuses newly transformed occupancy values while preserving verified previously mapped cells and managing unknown spaces.

### 3. Planner Node (`src/robot/planner`)
* **State Machine Architecture:** Operates with distinct internal states:
  * `WAITING_FOR_GOAL`: Listens for target coordinates published to `/goal_point`.
  * `PLANNING_AND_TRACKING`: Generates and monitors trajectory execution.
* **A\* Pathfinding Algorithm:** Employs an $A^*$ graph search algorithm on the global occupancy grid utilizing Euclidean distance heuristics to compute an optimal, minimum-cost collision-free path.
* **Dynamic Replanning:** Periodically triggers path recalculation upon map updates, obstacle interference with the active path, or goal changes.

### 4. Control Node (`src/robot/control`)
* **Pure Pursuit Path Following:** Computes steering curvature by tracking a lookahead target point along the planned trajectory at an adjustable distance $L_d$.
* **Differential Drive Kinematics:** Computes real-time linear ($v_x$) and angular ($\omega_z$) velocities:
  $$\kappa = \frac{2 \sin(\alpha)}{L_d}, \quad \omega_z = v_x \cdot \kappa$$
* **Topic Output:** Emits velocity commands to `/cmd_vel` to guide the robot along the computed path, stopping gracefully within goal tolerance thresholds.

---

## Tech Stack & Prerequisites

* **OS:** Linux (Ubuntu 22.04+ LTS), WSL2 (Windows), or macOS
* **Robotics Middleware:** ROS 2 Humble Hawksbill (`rclcpp`)
* **Simulation & Visualization:** Gazebo, Foxglove Studio
* **Containerization:** Docker Engine & Docker Compose
* **Build System:** CMake, `ament_cmake`, Colcon

---

## Repository Structure

```text
wato_asd_training/
├── config/                     # Foxglove layout and runtime configurations
├── docker/                     # Dockerfiles and environment recipes
├── src/
│   └── robot/
│       ├── costmap/            # 2D costmap generation & obstacle inflation
│       ├── map_memory/         # Odometry-driven global map stitching
│       ├── planner/            # A* global path planning state machine
│       └── control/            # Pure pursuit trajectory tracking controller
├── watod                       # Monorepo container orchestration script
├── watod-config.sh             # Active module configuration
└── README.md
```
