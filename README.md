# 🛣️ Trajectory Tracking with TurtleBot3 (ROS 2 Humble)

## 📦 Package: `trajectory_tracking`

This ROS 2 package provides an end-to-end trajectory tracking system for TurtleBot3. It enables a robot to move from its current pose to a user-specified goal pose through:

- ✅ Path planning based on a goal pose
- ✅ Spline-based path smoothing
- ✅ Time-stamped trajectory generation
- ✅ Pure Pursuit controller for trajectory tracking

Tested using **TurtleBot3** in **ROS 2 Humble**.

---

## 🧠 System Overview

```mermaid
graph TD
    A[/goal_pose (PoseStamped)] --> B[PathPlannerNode]
    subgraph PathPlannerNode
        B1[/odom (Odometry)]
    end
    B --> C[Smoothed Waypoints (Pose2D list)]
    C --> D[PurePursuitController]
    D1[/odom (Odometry)] --> D
    D --> E[/cmd_vel (Twist)]
    E --> F[TurtleBot3 Robot]

## 🚀 How to Build & Run

bash
cd ~/ros2_ws
colcon build --packages-select trajectory_tracking
source install/setup.bash
ros2 launch trajectory_tracking system.launch.py