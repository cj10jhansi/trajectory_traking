from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_tracking',
            executable='trajectory_tracking_node',  
            name='trajectory_tracker_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
