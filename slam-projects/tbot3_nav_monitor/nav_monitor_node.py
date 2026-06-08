#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class NavMonitorNode(Node):
    """
    TurtleBot3 Navigation Performance Monitor.

    Features:
    - /odom monitoring
    - /scan monitoring
    - /cmd_vel monitoring
    - CSV logging
    - live status topic
    - stuck/recovery detection
    - adaptive safe velocity publishing
    """

    def __init__(self):
        super().__init__('tbot3_nav_monitor')

        # Topics
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('log_directory', 'logs')

        # Metric parameters
        self.declare_parameter('battery_drain_per_meter', 2.0)
        self.declare_parameter('obstacle_warning_distance', 0.35)
        self.declare_parameter('complex_environment_threshold', 0.60)

        # Stuck detection parameters
        self.declare_parameter('cmd_velocity_threshold', 0.02)
        self.declare_parameter('actual_velocity_threshold', 0.01)
        self.declare_parameter('stuck_distance_threshold', 0.005)
        self.declare_parameter('stuck_time_threshold_sec', 5.0)

        # Adaptive behavior parameters
        self.declare_parameter('safe_velocity_scale', 0.5)
        self.declare_parameter('recovery_event_threshold', 3)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.log_directory = self.get_parameter('log_directory').value

        self.battery_drain_per_meter = float(
            self.get_parameter('battery_drain_per_meter').value
        )
        self.obstacle_warning_distance = float(
            self.get_parameter('obstacle_warning_distance').value
        )
        self.complex_environment_threshold = float(
            self.get_parameter('complex_environment_threshold').value
        )

        self.cmd_velocity_threshold = float(
            self.get_parameter('cmd_velocity_threshold').value
        )
        self.actual_velocity_threshold = float(
            self.get_parameter('actual_velocity_threshold').value
        )
        self.stuck_distance_threshold = float(
            self.get_parameter('stuck_distance_threshold').value
        )
        self.stuck_time_threshold_sec = float(
            self.get_parameter('stuck_time_threshold_sec').value
        )

        self.safe_velocity_scale = float(
            self.get_parameter('safe_velocity_scale').value
        )
        self.recovery_event_threshold = int(
            self.get_parameter('recovery_event_threshold').value
        )

        # Odometry state
        self.previous_x = None
        self.previous_y = None
        self.total_distance = 0.0
        self.battery_level = 100.0
        self.sample_count = 0

        # LaserScan state
        self.min_obstacle_distance = float('inf')
        self.front_obstacle_distance = float('inf')
        self.environment_complexity = 0.0
        self.close_obstacle_detected = False

        # Command velocity state
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.command_active = False

        # Adaptive velocity state
        self.adaptation_active = False
        self.safe_linear_velocity = 0.0
        self.safe_angular_velocity = 0.0

        # Stuck/recovery state
        self.stuck_start_time = None
        self.is_currently_stuck = False
        self.recovery_event_count = 0

        # CSV logger
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

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Publishers
        self.status_publisher = self.create_publisher(
            String,
            '/tbot3_nav_monitor/status',
            10
        )

        self.safe_cmd_vel_publisher = self.create_publisher(
            Twist,
            '/tbot3_nav_monitor/safe_cmd_vel',
            10
        )

        self.get_logger().info('TurtleBot3 Navigation Monitor started.')
        self.get_logger().info(f'Subscribed to odometry topic: {self.odom_topic}')
        self.get_logger().info(f'Subscribed to laser scan topic: {self.scan_topic}')
        self.get_logger().info(f'Subscribed to command velocity topic: {self.cmd_vel_topic}')
        self.get_logger().info('Publishing status topic: /tbot3_nav_monitor/status')
        self.get_logger().info('Publishing safe velocity topic: /tbot3_nav_monitor/safe_cmd_vel')
        self.get_logger().info(f'CSV log file: {self.csv_path}')

    def _setup_csv_logger(self):
        package_root = os.path.expanduser(
            '~/tbot3_nav_monitor_ws/src/tbot3_nav_monitor'
        )

        log_dir_path = os.path.join(package_root, self.log_directory)
        os.makedirs(log_dir_path, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            log_dir_path,
            f'nav_monitor_log_{timestamp}.csv'
        )

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'timestamp',
            'sample_count',
            'x_position_m',
            'y_position_m',
            'linear_velocity_mps',
            'cmd_linear_velocity_mps',
            'cmd_angular_velocity_rps',
            'safe_linear_velocity_mps',
            'safe_angular_velocity_rps',
            'command_active',
            'adaptation_active',
            'incremental_distance_m',
            'total_distance_m',
            'battery_level_percent',
            'min_obstacle_distance_m',
            'front_obstacle_distance_m',
            'environment_complexity',
            'close_obstacle_detected',
            'is_stuck',
            'recovery_event_count',
            'system_status'
        ])

        self.csv_file.flush()

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_linear = msg.linear.x
        self.last_cmd_angular = msg.angular.z

        self.command_active = (
            abs(self.last_cmd_linear) > self.cmd_velocity_threshold
            or abs(self.last_cmd_angular) > self.cmd_velocity_threshold
        )

        risk_detected = self.is_risk_detected()

        safe_msg = Twist()

        if risk_detected:
            self.adaptation_active = True
            safe_msg.linear.x = msg.linear.x * self.safe_velocity_scale
            safe_msg.angular.z = msg.angular.z * self.safe_velocity_scale
        else:
            self.adaptation_active = False
            safe_msg.linear.x = msg.linear.x
            safe_msg.angular.z = msg.angular.z

        self.safe_linear_velocity = safe_msg.linear.x
        self.safe_angular_velocity = safe_msg.angular.z

        self.safe_cmd_vel_publisher.publish(safe_msg)

    def scan_callback(self, msg: LaserScan):
        valid_ranges = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]

        if not valid_ranges:
            return

        self.min_obstacle_distance = min(valid_ranges)

        n = len(msg.ranges)
        center = n // 2
        sector_width = max(10, n // 12)

        front_ranges_raw = msg.ranges[
            max(0, center - sector_width): min(n, center + sector_width)
        ]

        front_ranges = [
            r for r in front_ranges_raw
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]

        if front_ranges:
            self.front_obstacle_distance = min(front_ranges)

        close_ranges = [
            r for r in valid_ranges
            if r < 1.0
        ]

        self.environment_complexity = len(close_ranges) / len(valid_ranges)

        self.close_obstacle_detected = (
            self.front_obstacle_distance < self.obstacle_warning_distance
        )

    def is_risk_detected(self):
        return (
            self.close_obstacle_detected
            or self.environment_complexity > self.complex_environment_threshold
            or self.recovery_event_count >= self.recovery_event_threshold
        )

    def update_stuck_detection(self, linear_velocity, incremental_distance, current_time_sec):
        low_actual_motion = (
            abs(linear_velocity) < self.actual_velocity_threshold
            and incremental_distance < self.stuck_distance_threshold
        )

        stuck_condition = self.command_active and low_actual_motion

        if stuck_condition:
            if self.stuck_start_time is None:
                self.stuck_start_time = current_time_sec

            stuck_duration = current_time_sec - self.stuck_start_time

            if stuck_duration >= self.stuck_time_threshold_sec:
                if not self.is_currently_stuck:
                    self.is_currently_stuck = True
                    self.recovery_event_count += 1
                    self.get_logger().warn(
                        f'Stuck/recovery event detected. '
                        f'Total events: {self.recovery_event_count}'
                    )
        else:
            self.stuck_start_time = None
            self.is_currently_stuck = False

    def get_system_status(self):
        if self.is_currently_stuck:
            return 'STUCK_RECOVERY_EVENT'

        if self.battery_level < 30.0:
            return 'LOW_BATTERY'

        if self.close_obstacle_detected:
            return 'OBSTACLE_TOO_CLOSE'

        if self.environment_complexity > self.complex_environment_threshold:
            return 'COMPLEX_ENVIRONMENT'

        if self.adaptation_active:
            return 'ADAPTIVE_SPEED_REDUCTION'

        return 'NORMAL'

    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now().to_msg()
        timestamp_sec = current_time.sec + current_time.nanosec * 1e-9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        linear_velocity = math.sqrt(vx ** 2 + vy ** 2)

        incremental_distance = 0.0

        if self.previous_x is not None and self.previous_y is not None:
            dx = x - self.previous_x
            dy = y - self.previous_y
            incremental_distance = math.sqrt(dx ** 2 + dy ** 2)
            self.total_distance += incremental_distance

        self.previous_x = x
        self.previous_y = y

        self.battery_level = max(
            0.0,
            100.0 - self.total_distance * self.battery_drain_per_meter
        )

        self.sample_count += 1

        self.update_stuck_detection(
            linear_velocity,
            incremental_distance,
            timestamp_sec
        )

        system_status = self.get_system_status()

        self.csv_writer.writerow([
            f'{timestamp_sec:.3f}',
            self.sample_count,
            f'{x:.4f}',
            f'{y:.4f}',
            f'{linear_velocity:.4f}',
            f'{self.last_cmd_linear:.4f}',
            f'{self.last_cmd_angular:.4f}',
            f'{self.safe_linear_velocity:.4f}',
            f'{self.safe_angular_velocity:.4f}',
            int(self.command_active),
            int(self.adaptation_active),
            f'{incremental_distance:.4f}',
            f'{self.total_distance:.4f}',
            f'{self.battery_level:.2f}',
            f'{self.min_obstacle_distance:.4f}',
            f'{self.front_obstacle_distance:.4f}',
            f'{self.environment_complexity:.4f}',
            int(self.close_obstacle_detected),
            int(self.is_currently_stuck),
            self.recovery_event_count,
            system_status
        ])

        self.csv_file.flush()

        status_msg = String()
        status_msg.data = (
            f'distance={self.total_distance:.2f},'
            f'battery={self.battery_level:.1f},'
            f'front_obstacle={self.front_obstacle_distance:.2f},'
            f'complexity={self.environment_complexity:.2f},'
            f'cmd_active={int(self.command_active)},'
            f'adaptation_active={int(self.adaptation_active)},'
            f'safe_scale={self.safe_velocity_scale:.2f},'
            f'is_stuck={int(self.is_currently_stuck)},'
            f'recovery_events={self.recovery_event_count},'
            f'status={system_status}'
        )
        self.status_publisher.publish(status_msg)

        if self.sample_count % 20 == 0:
            self.get_logger().info(
                f'x={x:.2f} m, y={y:.2f} m, '
                f'v={linear_velocity:.2f} m/s, '
                f'cmd_v={self.last_cmd_linear:.2f} m/s, '
                f'safe_v={self.safe_linear_velocity:.2f} m/s, '
                f'distance={self.total_distance:.2f} m, '
                f'battery={self.battery_level:.1f}%, '
                f'front_obstacle={self.front_obstacle_distance:.2f} m, '
                f'complexity={self.environment_complexity:.2f}, '
                f'adaptation={int(self.adaptation_active)}, '
                f'recovery_events={self.recovery_event_count}, '
                f'status={system_status}'
            )

        if self.adaptation_active and self.sample_count % 20 == 0:
            self.get_logger().warn(
                'Adaptive speed reduction active. '
                f'Velocity scale={self.safe_velocity_scale:.2f}'
            )

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = NavMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.csv_file is not None:
            node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
