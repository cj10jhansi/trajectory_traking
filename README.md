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

                    [ /goal_pose (geometry_msgs::PoseStamped) ]
                                      │
                                      ▼
                          ┌────────────────────────┐
                          │     PathPlannerNode     │
                          │  (Sub: /goal_pose, /odom)│
                          │  (Pub: waypoints topic) │
                          └────────────┬────────────┘
                                       │
                          Smoothed waypoints (std::vector<Pose2D>)
                                       ▼
                          ┌────────────────────────┐
                          │ PurePursuitController   │
                          │ (Sub: /odom + waypoints)│
                          │ (Pub: /cmd_vel)         │
                          └────────────┬────────────┘
                                       │
                                   [ /cmd_vel ]
                                       │
                                       ▼
                              [ TurtleBot3 Robot ]


## 🚀 How to Build & Run
        bash
        cd ~/ros2_ws
        colcon build --packages-select trajectory_tracking
        source install/setup.bash
        ros2 launch trajectory_tracking system.launch.py