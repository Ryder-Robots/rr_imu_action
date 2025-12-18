# Utilities

Test utilities for Ryder Robots ROS2 packages.

## rr_actions.py

Command-line utility for testing ROS2 actions across rr_* packages.

### Requirements

Ensure your ROS2 workspace is sourced:
```bash
source /home/aaron/ros2_ws/install/setup.bash
```

### Usage

#### IMU Action Testing

Test the IMU monitoring action with default settings:
```bash
python3 rr_actions.py imu
```

Test with custom frame ID and timeout:
```bash
python3 rr_actions.py imu --frame-id base_link --timeout 15.0
```

Test with custom action server name:
```bash
python3 rr_actions.py imu --action-name /robot1/imu_monitor_action
```

#### Help

View all available options:
```bash
python3 rr_actions.py --help
python3 rr_actions.py imu --help
```

### Adding New Action Types

To add support for additional action types:

1. Import the new action type at the top of the file:
   ```python
   from rr_interfaces.action import YourNewAction
   ```

2. Create a new action runner class inheriting from `ActionRunner`:
   ```python
   class YourActionRunner(ActionRunner):
       def __init__(self, node: Node, action_name: str, **kwargs):
           super().__init__(node, action_name)
           # Store any additional parameters

       def create_goal(self) -> YourNewAction.Goal:
           goal = YourNewAction.Goal()
           # Populate goal fields
           return goal

       def get_action_type(self) -> type:
           return YourNewAction
   ```

3. Add a run function for your action:
   ```python
   def run_your_action(args) -> int:
       rclpy.init()
       node = ActionTestNode()
       try:
           runner = YourActionRunner(node=node, ...)
           success = runner.send_goal(timeout_sec=args.timeout)
           return 0 if success else 1
       finally:
           node.destroy_node()
           rclpy.shutdown()
   ```

4. Add a subparser in `main()`:
   ```python
   your_parser = subparsers.add_parser('your_action', help='Test your action')
   your_parser.add_argument(...)
   your_parser.set_defaults(func=run_your_action)
   ```

### Architecture

The script uses an extensible object-oriented design:

- **`ActionRunner`**: Abstract base class providing common action client functionality
- **`ImuActionRunner`**: Concrete implementation for IMU monitoring actions
- **`ActionTestNode`**: Simple ROS2 node for running action tests
- **Command-line interface**: Uses argparse with subcommands for different action types

This design makes it easy to add new action types without modifying existing code.
