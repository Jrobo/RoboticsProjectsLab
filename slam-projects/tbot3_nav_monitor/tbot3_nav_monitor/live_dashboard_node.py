#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich.panel import Panel


class LiveDashboardNode(Node):
    """
    Terminal-based live dashboard for TurtleBot3 navigation monitoring.

    Subscribes to:
    - /tbot3_nav_monitor/status
    - /tbot3_nav_monitor/goal_metrics

    Displays:
    - distance
    - battery
    - obstacle distance
    - environment complexity
    - adaptive status
    - recovery count
    - goal error
    - execution time
    - path efficiency
    """

    def __init__(self):
        super().__init__('tbot3_live_dashboard')

        self.console = Console()
        self.status_data = {}
        self.goal_data = {}

        self.create_subscription(
            String,
            '/tbot3_nav_monitor/status',
            self.status_callback,
            10
        )

        self.create_subscription(
            String,
            '/tbot3_nav_monitor/goal_metrics',
            self.goal_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.update_dashboard)

        self.live = Live(
            self.render_dashboard(),
            console=self.console,
            refresh_per_second=2,
            screen=True
        )
        self.live.start()

        self.get_logger().info('Live dashboard started.')

    def parse_key_value_string(self, text):
        data = {}
        pairs = text.split(',')
        for pair in pairs:
            if '=' in pair:
                key, value = pair.split('=', 1)
                data[key.strip()] = value.strip()
        return data

    def status_callback(self, msg):
        self.status_data = self.parse_key_value_string(msg.data)

    def goal_callback(self, msg):
        self.goal_data = self.parse_key_value_string(msg.data)

    def update_dashboard(self):
        self.live.update(self.render_dashboard())

    def render_dashboard(self):
        table = Table(title='TurtleBot3 Navigation Monitor Live Dashboard')

        table.add_column('Metric', style='cyan', no_wrap=True)
        table.add_column('Value', style='green')

        table.add_row('Distance travelled', self.status_data.get('distance', 'N/A') + ' m')
        table.add_row('Battery level', self.status_data.get('battery', 'N/A') + ' %')
        table.add_row('Front obstacle', self.status_data.get('front_obstacle', 'N/A') + ' m')
        table.add_row('Environment complexity', self.status_data.get('complexity', 'N/A'))
        table.add_row('Command active', self.status_data.get('cmd_active', 'N/A'))
        table.add_row('Adaptation active', self.status_data.get('adaptation_active', 'N/A'))
        table.add_row('Safe velocity scale', self.status_data.get('safe_scale', 'N/A'))
        table.add_row('Is stuck', self.status_data.get('is_stuck', 'N/A'))
        table.add_row('Recovery events', self.status_data.get('recovery_events', 'N/A'))
        table.add_row('System status', self.status_data.get('status', 'N/A'))

        table.add_row('--- Goal metrics ---', '---')
        table.add_row('Goal count', self.goal_data.get('goal_count', 'N/A'))
        table.add_row('Completed goals', self.goal_data.get('completed_goals', 'N/A'))
        table.add_row('Goal active', self.goal_data.get('goal_active', 'N/A'))
        table.add_row('Goal completed', self.goal_data.get('goal_completed', 'N/A'))
        table.add_row('Goal error', self.goal_data.get('goal_error', 'N/A') + ' m')
        table.add_row('Final goal error', self.goal_data.get('final_goal_error', 'N/A') + ' m')
        table.add_row('Execution time', self.goal_data.get('execution_time', 'N/A') + ' s')
        table.add_row('Actual path', self.goal_data.get('actual_path', 'N/A') + ' m')
        table.add_row('Optimal path', self.goal_data.get('optimal_path', 'N/A') + ' m')
        table.add_row('Path efficiency', self.goal_data.get('path_efficiency', 'N/A'))

        note = (
            'Listening to /tbot3_nav_monitor/status and '
            '/tbot3_nav_monitor/goal_metrics. '
            '/safe_cmd_vel is an adaptive safety output, not a direct replacement for /cmd_vel.'
        )

        return Panel(table, subtitle=note)

    def destroy_node(self):
        try:
            self.live.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiveDashboardNode()

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
