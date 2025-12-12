# rr_imu_action

## Overview

`rr_imu_action` is a **ROS 2 Lifecycle Node** that provides a pluginlib-based action server interface for IMU (Inertial Measurement Unit) operations. This node acts as a thin wrapper that manages the ROS 2 action server lifecycle while delegating the actual IMU action logic to dynamically-loaded plugins.

## Architecture

This package uses a **plugin-based architecture** to separate concerns:

- **Node Responsibilities:**
  - ROS 2 lifecycle management (configure, activate, deactivate, cleanup)
  - Action server creation and lifecycle
  - Plugin loading and initialization via `pluginlib`
  - Forwarding action callbacks to plugin implementation

- **Plugin Responsibilities (RRImuActionPluginIface):**
  - Concrete IMU action implementation
  - Action goal handling: `handle_goal()`
  - Action cancellation: `handle_cancel()`
  - Action execution: `handle_accepted()`
  - Plugin-specific configuration

This design allows different IMU implementations to be swapped via configuration without code changes.

## Node Type

**IMPORTANT:** This is a **ROS 2 Lifecycle Node** (`rclcpp_lifecycle::LifecycleNode`)

The node must be transitioned through lifecycle states:
1. **Unconfigured** → `configure` → **Inactive**
2. **Inactive** → `activate` → **Active**
3. **Active** → `deactivate` → **Inactive**
4. **Inactive** → `cleanup` → **Unconfigured**

## Lifecycle States

### Unconfigured → Inactive (configure)

**Transition:** `on_configure()`

**Actions:**
- Loads the IMU action plugin using `pluginlib::ClassLoader`
- Initializes plugin via `plugin_lib_->on_configure()`
- Plugin name determined by ROS parameter `imu_action_plugin`

**Success Criteria:**
- Plugin loaded successfully
- Plugin configuration completed without errors

### Inactive → Active (activate)

**Transition:** `on_activate()`

**Actions:**
- Creates and activates the action server for `MonitorImuAction`
- Registers plugin callback functions:
  - `plugin_lib_->handle_goal()` - Goal acceptance/rejection
  - `plugin_lib_->handle_cancel()` - Cancellation requests
  - `plugin_lib_->handle_accepted()` - Goal execution

**Success Criteria:**
- Action server created and activated
- Plugin callbacks successfully registered

### Active → Inactive (deactivate)

**Transition:** `on_deactivate()`

**Actions:**
- Currently unimplemented (returns SUCCESS)
- Future: Deactivate action server, stop accepting new goals

### Inactive → Unconfigured (cleanup)

**Transition:** `on_cleanup()`

**Actions:**
- Currently unimplemented (returns SUCCESS)
- Future: Release plugin resources, clean up action server

## Actions

### MonitorImuAction

**Action Type:** `rr_interfaces/action/MonitorImuAction`

**Description:** Monitors IMU sensor data and provides feedback/results based on the plugin implementation.

**Action Interface:**
```
Goal:    (Defined in rr_interfaces/action/monitor_imu_action.action)
Result:  (Defined in rr_interfaces/action/monitor_imu_action.action)
Feedback: (Defined in rr_interfaces/action/monitor_imu_action.action)
```

**Note:** The specific goal, result, and feedback fields are defined by the action message in the `rr_interfaces` package.

## ROS 2 Parameters

| Parameter Name | Type | Default | Description |
|----------------|------|---------|-------------|
| `imu_action_plugin` | `string` | (default plugin) | Name of the plugin to load from the `rr_common_base` package. Must implement the `rrobots::interfaces::RRImuActionPluginIface` interface. |

### Example Parameter Configuration

```yaml
rr_imu_action_node:
  ros__parameters:
    imu_action_plugin: "my_custom_imu_plugin"
```

## Plugin Interface

Plugins must implement the `rrobots::interfaces::RRImuActionPluginIface` interface from the `rr_common_base` package.

### Required Plugin Methods

```cpp
namespace rrobots::interfaces {

class RRImuActionPluginIface {
public:
  // Configuration called during on_configure()
  virtual bool on_configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node) = 0;

  // Action goal callback
  virtual rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MonitorImuAction::Goal> goal) = 0;

  // Action cancel callback
  virtual rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle) = 0;

  // Action execution callback
  virtual void handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle) = 0;
};

}  // namespace rrobots::interfaces
```

## Usage

### Launching as a Lifecycle Node

```bash
ros2 run rr_imu_action rr_imu_action_node
```

### Lifecycle Management

**Configure the node:**
```bash
ros2 lifecycle set /rr_imu_action_node configure
```

**Activate the node:**
```bash
ros2 lifecycle set /rr_imu_action_node activate
```

**Check lifecycle state:**
```bash
ros2 lifecycle get /rr_imu_action_node
```

**Deactivate the node:**
```bash
ros2 lifecycle set /rr_imu_action_node deactivate
```

**Cleanup the node:**
```bash
ros2 lifecycle set /rr_imu_action_node cleanup
```

### Using the Action

**Send a goal (when node is Active):**
```bash
ros2 action send_goal /monitor_imu_action rr_interfaces/action/MonitorImuAction "{goal_fields_here}"
```

**List available actions:**
```bash
ros2 action list
```

**Get action info:**
```bash
ros2 action info /monitor_imu_action
```

## Dependencies

- **ROS 2** (Humble or later recommended)
- `rclcpp`
- `rclcpp_lifecycle`
- `rclcpp_action`
- `rclcpp_components`
- `pluginlib`
- `rr_common_base` - Provides the `RRImuActionPluginIface` interface
- `rr_interfaces` - Provides the `MonitorImuAction` action definition

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select rr_imu_action
source install/setup.bash
```

## Plugin Development

To create a custom IMU action plugin:

1. Create a class that inherits from `rrobots::interfaces::RRImuActionPluginIface`
2. Implement all required methods (`on_configure`, `handle_goal`, `handle_cancel`, `handle_accepted`)
3. Export the plugin using `PLUGINLIB_EXPORT_CLASS` macro
4. Register the plugin in your package's plugin XML file
5. Specify the plugin name via the `imu_action_plugin` parameter

### Example Plugin Registration

```xml
<library path="my_imu_plugin">
  <class type="my_namespace::MyImuPlugin"
         base_class_type="rrobots::interfaces::RRImuActionPluginIface">
    <description>My custom IMU action plugin</description>
  </class>
</library>
```

## License

MIT License - See LICENSE file for details.

## Author

Copyright (c) 2025 Ryder Robots
