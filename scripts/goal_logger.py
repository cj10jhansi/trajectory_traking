import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv

class GoalLogger(Node):
    def __init__(self):
        super().__init__('goal_logger')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # or '/navigate_to_pose/goal'
            self.goal_callback,
            10)
        self.waypoints = []

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        print(f"Logged waypoint: ({x}, {y})")
        self.waypoints.append((x, y))
        with open('waypoints.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow([x, y])

rclpy.init()
node = GoalLogger()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
