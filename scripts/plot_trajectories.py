#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import pandas as pd
import matplotlib.pyplot as plt
import os


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        self.declare_parameter('waypoints_csv', 'waypoints.csv')
        self.declare_parameter('smoothed_csv', 'smoothed.csv')
        self.declare_parameter('trajectory_csv', 'trajectory.csv')

        self.plot_trajectories()

    def plot_trajectories(self):
        waypoints_file = self.get_parameter('waypoints_csv').get_parameter_value().string_value
        smoothed_file = self.get_parameter('smoothed_csv').get_parameter_value().string_value
        trajectory_file = self.get_parameter('trajectory_csv').get_parameter_value().string_value

        try:
            waypoints = pd.read_csv(waypoints_file)
            smoothed = pd.read_csv(smoothed_file)
            trajectory = pd.read_csv(trajectory_file)

            plt.figure(figsize=(10, 6))
            plt.plot(waypoints['x'], waypoints['y'], 'o-', label='Original Waypoints', color='blue')
            plt.plot(smoothed['x'], smoothed['y'], 'x--', label='Smoothed Path', color='green')
            plt.plot(trajectory['x'], trajectory['y'], 's-', label='Time-Stamped Trajectory', color='red')

            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Trajectory Comparison')
            plt.legend()
            plt.grid(True)
            plt.axis('equal')

            plt.show()

            self.get_logger().info('Trajectory plot displayed successfully.')

        except Exception as e:
            self.get_logger().error(f'Failed to plot trajectories: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
