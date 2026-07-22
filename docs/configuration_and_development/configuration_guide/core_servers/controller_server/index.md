# Controller Server { #controller-server }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/main/nav2_controller).

The Controller Server implements the server for handling the controller requests for the stack and host a map of plugin implementations.
It will take in path and plugin names for controller, progress checker and goal checker to use and call the appropriate plugins.
It also hosts the local costmap.

## Parameters

### **`controller_frequency`**

Type: `double` Default: `20.0`

:   Frequency to run controller (Hz).

### **`costmap_update_timeout`**

Type: `double` Default: `0.3`

:   The timeout value (seconds) for the costmap to be fully updated before a control effort can be computed.

### **`use_realtime_priority`**

Type: `bool` Default: `false`

:   Adds soft real-time prioritization to the controller server to better ensure resources to time sensitive portions of the codebase. This will set the controller's execution thread to a higher priority than the rest of the system (`90`) to meet scheduling deadlines to have less missed loop rates. To use this feature, you use set the following inside of `/etc/security/limits.conf` to give userspace access to elevated prioritization permissions: `<username> soft rtprio 99 <username> hard rtprio 99`

### **`publish_zero_velocity`**

Type: `bool` Default: `true`

:   Whether to publish a zero velocity command on goal exit. This is useful for stopping the robot when a goal terminates.

### **`controller_plugins`**

Type: `vector<string>` Default: `["FollowPath"]`

:   List of mapped names for controller plugins for processing requests and parameters.

    Note
    :   Each plugin namespace defined in this list needs to have a `plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:

        ```yaml
        controller_server:
          ros__parameters:
            controller_plugins: ["FollowPath"]
            FollowPath:
              plugin: "dwb_core::DWBLocalPlanner"
        ```

### **`progress_checker_plugins`**

Type: `vector<string>` Default: `["progress_checker"]`

:   Mapped name for progress checker plugin for checking progress made by robot.

    Note
    :   The plugin namespace defined needs to have a `plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:
        ```yaml
        controller_server:
          ros__parameters:
            progress_checker_plugins: ["progress_checker"]
            progress_checker:
              plugin: "nav2_controller::SimpleProgressChecker"
        ```

### **`goal_checker_plugins`**

Type: `vector<string>` Default: `["goal_checker"]`

:   Mapped name for goal checker plugin for checking goal is reached. When the number of the plugins is more than 2, each `FollowPath` action needs to specify the goal checker plugin name with its `goal_checker_id` field.

    Note
    :   The plugin namespace defined needs to have a `plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:
        ```yaml
        controller_server:
          ros__parameters:
            goal_checker_plugins: ["goal_checker"]
            goal_checker:
              plugin: "nav2_controller::SimpleGoalChecker"
        ```

### **`path_handler_plugins`**

Type: `vector<string>` Default: `["PathHandler"]`

:   Mapped name for path handler plugin for processing path from the planner. When the number of the plugins is more than 2, each `FollowPath` action needs to specify the path handler plugin name with its `path_handler_id` field.

    Note
    :   The plugin namespace defined needs to have a `plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:
        ```yaml
        controller_server:
          ros__parameters:
            path_handler_plugins: ["PathHandler"]
            path_handler:
              plugin: "nav2_controller::FeasiblePathHandler"
        ```

### **`min_x_velocity_threshold`**

Type: `double` Default: `0.0001`

:   The controller server filters the velocity portion of the odometry messages received before sending them to the controller plugin.
    Odometry values below this threshold (in m/s) will be set to `0.0`.

### **`min_y_velocity_threshold`**

Type: `double` Default: `0.0001`

:   The controller server filters the velocity portion of the odometry messages received before sending them to the controller plugin.
    Odometry values below this threshold (in m/s) will be set to `0.0`. For non-holonomic robots

### **`min_theta_velocity_threshold`**

Type: `double` Default: `0.0001`

:   The controller server filters the velocity portion of the odometry messages received before sending them to the controller plugin.
    Odometry values below this threshold (in rad/s) will be set to `0.0`.

### **`failure_tolerance`**

Type: `double` Default: `0.0`

:   The maximum duration in seconds the called controller plugin can fail (i.e. the `computeVelocityCommands` function of the plugin throwing an exception) before the `nav2_msgs::action::FollowPath` action fails.
    Setting it to the special value of `-1.0` makes it infinite, `0` to disable, and any positive value for the appropriate timeout.

### **`speed_limit_topic`**

Type: `string` Default: `"speed_limit"`

:   Speed limiting topic name to subscribe. This could be published by Speed Filter (please refer to [Speed Filter Parameters][speed-filter-parameters] configuration page). You can also use this without the Speed Filter as well if you provide an external server to publish [these messages](https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/msg/SpeedLimit.msg).

### **`odom_topic`**

Type: `string` Default: `"odom"`

:   Topic to get instantaneous measurement of speed from.

### **`odom_duration`**

Type: `double` Default: `0.3`

:   Time (s) to buffer odometry commands to estimate the robot speed.

### **`enable_stamped_cmd_vel`**

Type: `bool` Default: `true`

:   Whether to use `geometry_msgs::msg::Twist` or `geometry_msgs::msg::TwistStamped` velocity data.
    `true` uses `TwistStamped`, `false` uses `Twist`.

### **`bond_heartbeat_period`**

Type: `double` Default: `0.25`

:   The lifecycle node bond mechanism publishing period (on the `/bond` topic). Disabled if inferior or equal to `0.0`.

### **`introspection_mode`**

Type: `string` Default: `"disabled"`

:   The introspection mode for services and actions. Options are `"disabled"`, `"metadata"`, `"contents"`.

### **`allow_parameter_qos_overrides`**

Type: `bool` Default: `true`

:   Whether to allow QoS profiles to be overwritten with parameterized values.

### **`search_window`**

Type: `double` Default: `2.0`

:   How far (in meters) along the path the searching algorithm will look for the closest point.

## Provided Plugins

The plugins listed below are inside the `nav2_controller` namespace.

<div class="grid" markdown>

[SimpleProgressChecker][simple-progress-checker]{ .md-button .md-button--primary }
[PoseProgressChecker][pose-progress-checker]{ .md-button .md-button--primary }
[AxisGoalChecker][axis-goal-checker]{ .md-button .md-button--primary }
[AdaptiveToleranceGoalChecker][adaptive-tolerance-goal-checker]{ .md-button .md-button--primary }
[SimpleGoalChecker][simple-goal-checker]{ .md-button .md-button--primary }
[StoppedGoalChecker][stopped-goal-checker]{ .md-button .md-button--primary }
[PositionGoalChecker][position-goal-checker]{ .md-button .md-button--primary }
[FeasiblePathHandler][feasible-path-handler]{ .md-button .md-button--primary }

</div>

## Default Plugins

When the `progress_checker_plugins`, `goal_checker_plugin`, `path_handler_plugin` or `controller_plugins` parameters are not overridden, the following default plugins are loaded:

| Namespace          | Plugin                                   |
|--------------------|------------------------------------------|
| "progress_checker" | "nav2_controller::SimpleProgressChecker" |
| "goal_checker"     | "nav2_controller::SimpleGoalChecker"     |
| "path_handler"     | "nav2_controller::FeasiblePathHandler"   |
| "FollowPath"       | "dwb_core::DWBLocalPlanner"              |

## Example

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    costmap_update_timeout: 0.3
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    odom_topic: "odom"
    odom_duration: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    path_handler_plguins: ["PathHandler"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      path_length_tolerance: 1.0
      stateful: True
    PathHandler:
      plugin: "nav2_controller::FeasiblePathHandler"
      prune_distance: 2.0
      enforce_path_inversion: True
      enforce_path_rotation: False
      inversion_xy_tolerance: 0.2
      inversion_yaw_tolerance: 0.4
      minimum_rotation_angle: 0.785
      reject_unit_path: False
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
```
