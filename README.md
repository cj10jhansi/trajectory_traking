# 🛣️ Trajectory Tracking with TurtleBot3 (ROS 2 Humble)

## 📦 Package: `trajectory_tracking`

Implements:
- Path Smoothing
- Time-stamped Trajectory Generation
- Pure Pursuit Controller
- Tested in TurtleBot3 Simulation

---

## 🚀 How to Build & Run

```bash
cd ~/ros2_ws
colcon build --packages-select trajectory_tracking
source install/setup.bash
ros2 launch trajectory_tracking system.launch.py

## System Design

### Modules:
- PathSmoother: Cubic Spline
- TrajectoryGenerator: Time stamp & velocity profile
- Controller: Pure Pursuit based on lookahead

### ROS 2 Nodes & Flow:

        [Waypoints] ---> [Smoother] ---> [Trajectory] ---> [Controller] ---> [cmd_vel]

Each node uses ROS 2 pub/sub and parameters.