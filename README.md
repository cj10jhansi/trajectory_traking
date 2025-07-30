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

## 🚀 system Design
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