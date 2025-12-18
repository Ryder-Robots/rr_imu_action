# Copyright (c) 2025 Ryder Robots
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
ROS2 Action Test Utility for Ryder Robots

This script provides a command-line interface for testing ROS2 actions in the
rr_* packages. Currently supports IMU monitoring actions, with extensibility
for additional action types.

Usage:
    python3 rr_actions.py imu --frame-id base_link --timeout 10.0
    python3 rr_actions.py --help
"""

import argparse
import sys
from abc import ABC, abstractmethod
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rr_interfaces.action import MonitorImuAction
from std_msgs.msg import Header


class ActionRunner(ABC):
    """Base class for action runners with common functionality."""

    def __init__(self, node: Node, action_name: str):
        self.node = node
        self.action_name = action_name
        self.result = None
        self.feedback_list = []

    @abstractmethod
    def create_goal(self) -> object:
        """Create and return the goal message for this action."""
        pass

    @abstractmethod
    def get_action_type(self) -> type:
        """Return the action type class."""
        pass

    def feedback_callback(self, feedback_msg):
        """Store feedback messages."""
        self.feedback_list.append(feedback_msg.feedback)
        self.node.get_logger().info(f'Feedback received: {feedback_msg.feedback}')

    def send_goal(self, timeout_sec: float = 10.0) -> bool:
        """Send goal and wait for result."""
        action_client = ActionClient(
            self.node,
            self.get_action_type(),
            self.action_name
        )

        self.node.get_logger().info(f'Waiting for action server: {self.action_name}')
        if not action_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error(
                f'Action server {self.action_name} not available after {timeout_sec}s'
            )
            return False

        goal_msg = self.create_goal()
        self.node.get_logger().info(f'Sending goal: {goal_msg}')

        send_goal_future = action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error('Goal rejected by action server')
            return False

        self.node.get_logger().info('Goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        self.result = result_future.result().result
        self.node.get_logger().info(f'Result received: {self.result}')

        return self.result.success if hasattr(self.result, 'success') else True


class ImuActionRunner(ActionRunner):
    """Runner for IMU monitoring actions."""

    def __init__(self, node: Node, action_name: str, frame_id: str):
        super().__init__(node, action_name)
        self.frame_id = frame_id

    def create_goal(self) -> MonitorImuAction.Goal:
        """Create IMU monitoring goal message."""
        goal = MonitorImuAction.Goal()
        goal.header = Header()
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        return goal

    def get_action_type(self) -> type:
        """Return MonitorImuAction type."""
        return MonitorImuAction


class ActionTestNode(Node):
    """ROS2 node for testing actions."""

    def __init__(self):
        super().__init__('rr_action_test_node')


def run_imu_action(args) -> int:
    """Run IMU monitoring action test."""
    rclpy.init()
    node = ActionTestNode()

    try:
        runner = ImuActionRunner(
            node=node,
            action_name=args.action_name,
            frame_id=args.frame_id
        )

        success = runner.send_goal(timeout_sec=args.timeout)

        if success:
            node.get_logger().info('Action completed successfully')
            if runner.result and hasattr(runner.result, 'imu'):
                node.get_logger().info(f'IMU data: {runner.result.imu}')
            return 0
        else:
            node.get_logger().error('Action failed')
            return 1

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
        return 130
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='ROS2 Action Test Utility for Ryder Robots',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Test IMU action with default settings:
    python3 rr_actions.py imu

  Test IMU action with custom frame and timeout:
    python3 rr_actions.py imu --frame-id base_link --timeout 15.0

  Test IMU action with custom action name:
    python3 rr_actions.py imu --action-name /custom/imu_monitor_action
        """
    )

    subparsers = parser.add_subparsers(dest='command', help='Action type to test')
    subparsers.required = True

    # IMU action subcommand
    imu_parser = subparsers.add_parser(
        'imu',
        help='Test IMU monitoring action'
    )
    imu_parser.add_argument(
        '--action-name',
        type=str,
        default='imu_monitor_action',
        help='Action server name (default: imu_monitor_action)'
    )
    imu_parser.add_argument(
        '--frame-id',
        type=str,
        default='imu_link',
        help='Frame ID for IMU data (default: imu_link)'
    )
    imu_parser.add_argument(
        '--timeout',
        type=float,
        default=10.0,
        help='Timeout in seconds to wait for action server (default: 10.0)'
    )
    imu_parser.set_defaults(func=run_imu_action)

    # Parse arguments
    args = parser.parse_args()

    # Execute the selected command
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
