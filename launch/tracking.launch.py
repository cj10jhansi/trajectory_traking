from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Your trajectory tracking node
        Node(
            package='trajectory_tracking',
            executable='trajectory_tracking_node',
            # name='trajectory_tracking_node',
            output='screen',
            parameters=[{
                'some_param': 'value'  # <-- Replace with actual params if needed
            }],
        ),
    ])
