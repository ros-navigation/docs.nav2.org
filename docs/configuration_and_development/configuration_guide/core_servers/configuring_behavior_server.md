# Behavior Server { #behavior-server }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_behaviors).

The Behavior Server implements the server for handling various behavior, such as recoveries and docking, requests and hosting a vector of plugins implementing various C++ behaviors.
It is also possible to implement independent behavior servers for each custom behavior, but this server will allow multiple behaviors to share resources such as costmaps and TF buffers to lower incremental costs for new behaviors.

Note: the wait recovery behavior has no parameters, the duration to wait is given in the action request.
Note: pre-Rolling/Humble this was the Recovery server, not behavior server. Launch files, behaviors and tests were all renamed.

## Behavior Server Parameters

### **`local_costmap_topic`**

Type: `string` Default: `"local_costmap/costmap_raw"`

:   Raw costmap topic for collision checking on the local costmap.

### **`global_costmap_topic`**

Type: `string` Default: `"global_costmap/costmap_raw"`

:   Raw costmap topic for collision checking on the global costmap.

### **`local_footprint_topic`**

Type: `string` Default: `"local_costmap/published_footprint"`

:   Topic for footprint in the local costmap frame.

### **`global_footprint_topic`**

Type: `string` Default: `"global_costmap/published_footprint"`

:   Topic for footprint in the global costmap frame.

### **`cycle_frequency`**

Type: `double` Default: `10.0`

:   Frequency to run behavior plugins.

### **`action_server_result_timeout`**

Type: `double` Default: `10.0` Unit: `seconds`

:   The timeout value (in seconds) for action servers to discard a goal handle if a result has not been produced. This used to default to 15 minutes in rcl but was changed to 10 seconds in this [PR #1012](https://github.com/ros2/rcl/pull/1012), which may be less than some actions in Nav2 take to run. For most applications, this should not need to be adjusted as long as the actions within the server do not exceed this deadline. This issue has been raised with OSRF to find another solution to avoid active goal timeouts for bookkeeping, so this is a semi-temporary workaround.


### **`transform_tolerance`**

Type: `double` Default: `0.1`

:   TF transform tolerance.

### **`local_frame`**

Type: `string` Default: `"odom"`

:   Local reference frame.

### **`global_frame`**

Type: `string` Default: `"map"`

:   Global reference frame.

### **`robot_base_frame`**

Type: `string` Default: `"base_link"`

:   Robot base frame.

### **`behavior_plugins`**

Type: `vector<string>` Default: `["spin", "backup", "drive_on_heading", "wait"]`

:   List of plugin names to use, also matches action server names.

    Note
    :   Each plugin namespace defined in this list needs to have a `plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:

        ```yaml
        behavior_server:
          ros__parameters:
            behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
            spin:
              plugin: "nav2_behaviors::Spin" # In Iron and older versions, "/" was used instead of "::"
            backup:
              plugin: "nav2_behaviors::BackUp" # In Iron and older versions, "/" was used instead of "::"
            drive_on_heading:
              plugin: "nav2_behaviors::DriveOnHeading" # In Iron and older versions, "/" was used instead of "::"
            wait:
              plugin: "nav2_behaviors::Wait" # In Iron and older versions, "/" was used instead of "::"
        ```

## Default Plugins

!!! note

    In Iron and older versions, "/" was used instead of "::".

When the `behavior_plugins` parameter is not overridden, the following default plugins are loaded:

| Namespace          | Plugin                           |
|--------------------|----------------------------------|
| "spin"             | "nav2_behaviors::Spin"           |
| "backup"           | "nav2_behaviors::BackUp"         |
| "drive_on_heading" | "nav2_behaviors::DriveOnHeading" |
| "wait"             | "nav2_behaviors::Wait"           |

## Spin Behavior Parameters

Spin distance is given from the action request

### **`simulate_ahead_time`**

Type: `double` Default: `2.0`

:   Time to look ahead for collisions (s).

### **`max_rotational_vel`**

Type: `double` Default: `1.0`

:   Maximum rotational velocity (rad/s).

### **`min_rotational_vel`**

Type: `double` Default: `0.4`

:   Minimum rotational velocity (rad/s).

### **`rotational_acc_lim`**

Type: `double` Default: `3.2`

:   Maximum rotational acceleration (rad/s^2).

### **`enable_stamped_cmd_vel`**

Type: `bool` Default: `false`

:   Whether to use `geometry_msgs::msg::Twist` or `geometry_msgs::msg::TwistStamped` velocity data. `true` uses `TwistStamped`, `false` uses `Twist`.

    Note
    :   This parameter is default `false` in Jazzy or older.

## BackUp Behavior Parameters

Backup distance, speed and time_allowance is given from the action request.

### **`simulate_ahead_time`**

Type: `double` Default: `2.0`

:   Time to look ahead for collisions (s).

### **`enable_stamped_cmd_vel`**

Type: `bool` Default: `false`

:   Whether to use `geometry_msgs::msg::Twist` or `geometry_msgs::msg::TwistStamped` velocity data. `true` uses `TwistStamped`, `false` uses `Twist`.

    Note
    :   This parameter is default `false` in Jazzy or older.

## DriveOnHeading Behavior Parameters

DriveOnHeading distance, speed and time_allowance is given from the action request.

### **`simulate_ahead_time`**

Type: `double` Default: `2.0`

:   Time to look ahead for collisions (s).

### **`enable_stamped_cmd_vel`**

Type: `bool` Default: `false`

:   Whether to use `geometry_msgs::msg::Twist` or `geometry_msgs::msg::TwistStamped` velocity data. `true` uses `TwistStamped`, `false` uses `Twist`.

    Note
    :   This parameter is default `false` in Jazzy or older.

### **`bond_heartbeat_period`**

Type: `double` Default: `0.1`

:   The lifecycle node bond mechanism publishing period (on the `/bond` topic). Disabled if inferior or equal to `0.0`.

## AssistedTeleop Behavior Parameters

AssistedTeleop time_allowance is given in the action request

### **`projection_time`**

Type: `double` Default: `1.0`

:   Time to look ahead for collisions (s).

### **`simulation_time_step`**

Type: `double` Default: `0.1`

:   Time step for projections (s).

### **`cmd_vel_teleop`**

Type: `string` Default: `"cmd_vel_teleop"`

:   Topic to listen for teleop messages.

### **`enable_stamped_cmd_vel`**

Type: `bool` Default: `false`

:   Whether to use `geometry_msgs::msg::Twist` or `geometry_msgs::msg::TwistStamped` velocity data. `true` uses `TwistStamped`, `false` uses `Twist`.

    Note
    :   This parameter is default `false` in Jazzy or older.

## Example

```yaml
behavior_server:
  ros__parameters:
    local_costmap_topic: local_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_costmap_topic: global_costmap/costmap_raw
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "assisted_teleop"]
    spin:
      plugin: "nav2_behaviors::Spin" # In Iron and older versions, "/" was used instead of "::"
    backup:
      plugin: "nav2_behaviors::BackUp" # In Iron and older versions, "/" was used instead of "::"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading" # In Iron and older versions, "/" was used instead of "::"
    wait:
      plugin: "nav2_behaviors::Wait" # In Iron and older versions, "/" was used instead of "::"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop" # In Iron and older versions, "/" was used instead of "::"
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
    enable_stamped_cmd_vel: false # False for Jazzy or older by default
```
