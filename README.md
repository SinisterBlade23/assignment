# Trajectory Tracking System - ROS2


---

## Overview

This ROS2 workspace implements a complete trajectory tracking system for differential drive robots using turtlesim. The system consists of three independent packages that communicate via ROS2 topics.

## System Architecture



## Packages

### 1. **waypoint_generator**
- **Purpose:** Generates 5 random waypoints within turtlesim bounds (1-10 in x,y)
- **Publishes:** `/waypoints` (geometry_msgs/PoseArray)
- **Node:** `waypoint_node`

### 2. **path_smoother**
- **Purpose:** Smooths discrete waypoints using cubic splines, generates time-parameterized trajectory
- **Subscribes:** `/waypoints` (geometry_msgs/PoseArray)
- **Publishes:** `/trajectory` (nav_msgs/Path)
- **Node:** `smoother_node`
- **Algorithm:** Cubic splines with constant velocity profile

### 3. **trajectory_tracker**
- **Purpose:** PID that makes turtle follow the trajectory
- **Subscribes:** 
  - `/trajectory` (nav_msgs/Path)
  - `/turtle1/pose` (turtlesim/Pose)
- **Publishes:** `/turtle1/cmd_vel` (geometry_msgs/Twist)
- **Node:** `tracker_node`
- **Controller:** PID control on heading and distance

---

## Installation

### Prerequisites
```bash
# Ubuntu 22.04 with ROS2 jazzy (or similar)
sudo apt update
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-turtlesim
sudo apt install python3-pip

# Python dependencies
pip3 install numpy scipy
```

### Build Workspace
```bash
cd ~/assignment_ws
colcon build
source install/setup.bash
```

---

## Usage

### Option 1: Launch Everything at Once (Recommended)
```bash
source /opt/ros/jazzy/setup.bash
cd ~/assignment_ws
source install/setup.bash
ros2 launch src/launch/trajectory_system.launch.py
```

### Option 2: Run Nodes Individually (For Debugging)

**Terminal 1: Turtlesim**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2: Waypoint Generator**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/assignment_ws
source install/setup.bash
ros2 run waypoint_generator waypoint_node
```

**Terminal 3: Path Smoother**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/assignment_ws
source install/setup.bash
ros2 run path_smoother smoother_node
```

**Terminal 4: Trajectory Tracker**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/assignment_ws
source install/setup.bash
ros2 run trajectory_tracker tracker_node
```

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/waypoints` | geometry_msgs/PoseArray | Random waypoints from generator |
| `/trajectory` | nav_msgs/Path | Smoothed trajectory from splines |
| `/turtle1/pose` | turtlesim/Pose | Current turtle position |
| `/turtle1/cmd_vel` | geometry_msgs/Twist | Velocity commands to turtle |

---

## Algorithms

### Path Smoothing (Cubic Splines)
- **Library:** `scipy.interpolate.CubicSpline`
- **Method:** Parametric interpolation of x(t) and y(t) separately
- **Properties:** C² continuous (smooth position, velocity, acceleration)
- **Sampling:** 200 points along spline for high resolution

### Trajectory Generation (Constant Velocity)
1. Sample spline at high resolution
2. Calculate cumulative arc length
3. Assign timestamps: `t = distance / velocity`
4. Velocity: 1.0 m/s (configurable)

### PID Controller 
- **Linear velocity:** `v = kp_lin × error + Ki_lin × ∫error + Kd_lin Ki_lin × ∫error + Kd_lin × d(error)/dtdistance_to_target` 
- **Angular velocity:** `ω = Kp_ang × angle_error + Ki_ang × ∫angle_error + Kd_ang × d(angle_error)/dt` (saturated at ±2.0 rad/s)
- **Gains:** `Tuned for smooth trajectory`
- **Goal tolerance:** 0.2 meters
- **Behavior:** Smooth approach with integral windup prevention and derivative filtering

---

## Design Decisions

### Why Three Separate Packages?
- **Modularity:** Each component is independently testable
- **ROS2 Best Practice:** Separation of concerns
- **Reusability:** Packages can be used in other projects
- **Debugging:** Easy to isolate issues

### Why Cubic Splines?
- C² continuous paths (smooth acceleration)
- Industry standard for robotics
- Efficient implementation in SciPy
- Better than linear interpolation

### Why Constant Velocity?
- Simplifies trajectory generation
- Predictable timing
- Sufficient for demonstration
- Easy baseline for future improvements

### Why Proportional Controller?
- Simplest effective controller
- No tuning of integral/derivative terms
- Fast convergence for smooth trajectories
- Easy to understand and debug

---

## Testing

### Manual Testing
```bash
# Check topics
ros2 topic list
ros2 topic echo /waypoints
ros2 topic echo /trajectory

# Monitor turtle pose
ros2 topic echo /turtle1/pose

# Monitor velocity commands
ros2 topic echo /turtle1/cmd_vel
```

### Debugging
```bash
# ROS2 node info
ros2 node list
ros2 node info /waypoint_generator
ros2 node info /path_smoother
ros2 node info /trajectory_tracker

# Topic info
ros2 topic info /waypoints
ros2 topic hz /turtle1/cmd_vel
```

---

## Extension to Real Robot

### Steps Required:
1. **Replace turtlesim with robot driver**
   - Subscribe to `/odom` for pose feedback
   - Publish to robot's `/cmd_vel`

2. **Add localization**
   - Use `robot_localization` package
   - Integrate wheel odometry + IMU + GPS
   - AMCL for map-based localization

3. **Sensor integration**
   - Add LIDAR/camera for obstacle detection
   - Safety: emergency stop on collision risk
   - Watchdog: timeout on velocity commands

4. **Controller tuning**
   - Adjust gains for robot's dynamics
   - Account for wheel slippage
   - Add velocity smoothing/filtering
   - Consider acceleration limits


---

## Obstacle Avoidance Extension (Extra Credit)

### Recommended: Dynamic Window Approach (DWA)

**Why DWA?**
- Real-time performance
- Considers robot kinematics and dynamics
- Proven in ROS navigation stack
- Suitable for differential drive

**Implementation:**
1. Sample admissible velocity space (v, ω)
2. Simulate short trajectories forward
3. Score each trajectory:
   - Obstacle clearance (maximize)
   - Heading to goal (minimize deviation)
   - Path following (minimize cross-track error)
4. Select best velocity, execute for one timestep
5. Repeat at control rate

**Alternative:** Artificial Potential Fields
- Attractive force toward goal
- Repulsive forces from obstacles
- Combine with base controller

---

## Directory Structure

```
assignment_ws/
├── src/
│   ├── waypoint_generator/
│   │   ├── waypoint_generator/
│   │   │   ├── __init__.py
│   │   │   └── waypoint_node.py
│   │   ├── resource/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   ├── path_smoother/
│   │   ├── path_smoother/
│   │   │   ├── __init__.py
│   │   │   └── smoother_node.py
│   │   ├── resource/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   ├── trajectory_tracker/
│   │   ├── trajectory_tracker/
│   │   │   ├── __init__.py
│   │   │   └── tracker_node.py
│   │   ├── resource/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   └── launch/
│       └── trajectory_system.launch.py
├── build/
├── install/
└── log/
```

---

## Troubleshooting

### Turtle not moving
- Check if all nodes are running: `ros2 node list`
- Verify trajectory published: `ros2 topic echo /trajectory`
- Check velocity commands: `ros2 topic echo /turtle1/cmd_vel`

### Waypoints not generating
- Make sure waypoint_generator is running
- Check topic: `ros2 topic echo /waypoints`

### Build errors
```bash
# Clean build
cd ~/assignment_ws
rm -rf build/ install/ log/
colcon build
```

### Python import errors
```bash
pip3 install numpy scipy
```

---

## AI Tools Used

- **Claude/GPT:** Implementing cubic smoothening and plots along with README.md

---

