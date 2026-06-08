#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class NavGoalMetricsNode(Node):
    """
    Goal-based navigation performance monitor.

    This node estimates:
    - goal execution time
    - final goal error
    - actual path length during a goal
    - straight-line optimal path length
    - path efficiency

    It listens to:
    - /goal_pose
    - /odom

    It publishes:
    - /tbot3_nav_monitor/goal_metrics
    """

    def __init__(self):
        super().__init__('tbot3_nav_goal_metrics')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('goal_tolerance_m', 0.25)
        self.declare_parameter('log_directory', 'logs')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)
        self.log_directory = self.get_parameter('log_directory').value

        # Robot pose
        self.current_x = None
        self.current_y = None
        self.previous_x = None
        self.previous_y = None

        # Goal state
        self.goal_active = False
        self.goal_completed = False
        self.goal_x = None
        self.goal_y = None
        self.goal_start_x = None
        self.goal_start_y = None
        self.goal_start_time = None

        # Metrics
        self.path_length_during_goal = 0.0
        self.straight_line_distance = 0.0
        self.current_goal_error = 0.0
        self.final_goal_error = 0.0
        self.goal_execution_time = 0.0
        self.path_efficiency = 0.0
        self.goal_count = 0
        self.completed_goal_count = 0

        # CSV
        self.csv_file = None
        self.csv_writer = None
        self._setup_csv_logger()

        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10
        )

        # Publisher
        self.metrics_publisher = self.create_publisher(
            String,
            '/tbot3_nav_monitor/goal_metrics',
            10
        )

        self.get_logger().info('TurtleBot3 Goal Metrics Monitor started.')
        self.get_logger().info(f'Subscribed to odometry topic: {self.odom_topic}')
        self.get_logger().info(f'Subscribed to goal topic: {self.goal_topic}')
        self.get_logger().info('Publishing goal metrics topic: /tbot3_nav_monitor/goal_metrics')
        self.get_logger().info(f'CSV goal metrics log file: {self.csv_path}')

    def _setup_csv_logger(self):
        package_root = os.path.expanduser(
            '~/tbot3_nav_monitor_ws/src/tbot3_nav_monitor'
        )

        log_dir_path = os.path.join(package_root, self.log_directory)
        os.makedirs(log_dir_path, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            log_dir_path,
            f'goal_metrics_log_{timestamp}.csv'
        )

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'timestamp',
            'goal_count',
            'completed_goal_count',
            'goal_active',
            'goal_completed',
            'start_x_m',
            'start_y_m',
            'goal_x_m',
            'goal_y_m',
            'current_x_m',
            'current_y_m',
            'current_goal_error_m',
            'final_goal_error_m',
            'goal_execution_time_sec',
            'actual_path_length_m',
            'straight_line_distance_m',
            'path_efficiency'
        ])
        self.csv_file.flush()

    def goal_callback(self, msg: PoseStamped):
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn(
                'Received goal, but current odometry is not available yet.'
            )
            return

        self.goal_count += 1
        self.goal_active = True
        self.goal_completed = False

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        self.goal_start_x = self.current_x
        self.goal_start_y = self.current_y

        self.goal_start_time = self.get_clock().now().nanoseconds / 1e9

        self.path_length_during_goal = 0.0
        self.final_goal_error = 0.0
        self.goal_execution_time = 0.0
        self.path_efficiency = 0.0

        self.straight_line_distance = math.sqrt(
            (self.goal_x - self.goal_start_x) ** 2
            + (self.goal_y - self.goal_start_y) ** 2
        )

        self.get_logger().info(
            f'New navigation goal received: '
            f'goal=({self.goal_x:.2f}, {self.goal_y:.2f}), '
            f'start=({self.goal_start_x:.2f}, {self.goal_start_y:.2f}), '
            f'optimal_distance={self.straight_line_distance:.2f} m'
        )

    def odom_callback(self, msg: Odometry):
        current_time_sec = self.get_clock().now().nanoseconds / 1e9

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.previous_x is not None and self.previous_y is not None:
            dx = self.current_x - self.previous_x
            dy = self.current_y - self.previous_y
            step_distance = math.sqrt(dx ** 2 + dy ** 2)

            if self.goal_active:
                self.path_length_during_goal += step_distance

        self.previous_x = self.current_x
        self.previous_y = self.current_y

        if self.goal_active and self.goal_x is not None and self.goal_y is not None:
            self.current_goal_error = math.sqrt(
                (self.goal_x - self.current_x) ** 2
                + (self.goal_y - self.current_y) ** 2
            )

            elapsed_time = current_time_sec - self.goal_start_time

            if self.current_goal_error <= self.goal_tolerance_m:
                self.goal_active = False
                self.goal_completed = True
                self.completed_goal_count += 1

                self.final_goal_error = self.current_goal_error
                self.goal_execution_time = elapsed_time

                if self.path_length_during_goal > 0.0:
                    self.path_efficiency = (
                        self.straight_line_distance / self.path_length_during_goal
                    )
                else:
                    self.path_efficiency = 0.0

                self.get_logger().info(
                    f'Goal completed. '
                    f'execution_time={self.goal_execution_time:.2f} s, '
                    f'final_error={self.final_goal_error:.3f} m, '
                    f'actual_path={self.path_length_during_goal:.2f} m, '
                    f'optimal_path={self.straight_line_distance:.2f} m, '
                    f'path_efficiency={self.path_efficiency:.3f}'
                )

        self.publish_and_log_metrics(current_time_sec)

    def publish_and_log_metrics(self, timestamp_sec):
        msg = String()
        msg.data = (
            f'goal_count={self.goal_count},'
            f'completed_goals={self.completed_goal_count},'
            f'goal_active={int(self.goal_active)},'
            f'goal_completed={int(self.goal_completed)},'
            f'goal_error={self.current_goal_error:.3f},'
            f'final_goal_error={self.final_goal_error:.3f},'
            f'execution_time={self.goal_execution_time:.2f},'
            f'actual_path={self.path_length_during_goal:.2f},'
            f'optimal_path={self.straight_line_distance:.2f},'
            f'path_efficiency={self.path_efficiency:.3f}'
        )
        self.metrics_publisher.publish(msg)

        self.csv_writer.writerow([
            f'{timestamp_sec:.3f}',
            self.goal_count,
            self.completed_goal_count,
            int(self.goal_active),
            int(self.goal_completed),
            self._fmt(self.goal_start_x),
            self._fmt(self.goal_start_y),
            self._fmt(self.goal_x),
            self._fmt(self.goal_y),
            self._fmt(self.current_x),
            self._fmt(self.current_y),
            f'{self.current_goal_error:.4f}',
            f'{self.final_goal_error:.4f}',
            f'{self.goal_execution_time:.4f}',
            f'{self.path_length_during_goal:.4f}',
            f'{self.straight_line_distance:.4f}',
            f'{self.path_efficiency:.4f}',
        ])
        self.csv_file.flush()

    def _fmt(self, value):
        if value is None:
            return ''
        return f'{value:.4f}'

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalMetricsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
