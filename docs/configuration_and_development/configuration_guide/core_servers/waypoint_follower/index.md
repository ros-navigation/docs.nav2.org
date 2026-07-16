# Waypoint Follower { #waypoint-follower }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/main/nav2_waypoint_follower).

The Waypoint Follower module implements a way of doing waypoint following using the NavigateToPose action server.
It will take in a set of ordered waypoints to follow and then try to navigate to them in order.
It also hosts a waypoint task executor plugin which can be used to perform custom behavior at a waypoint like waiting for user instruction, taking a picture, or picking up a box.
If a waypoint is not achievable, the `stop_on_failure` parameter will determine whether to continue to the next point or stop.

## Parameters

### **`stop_on_failure`**

Type: `bool` Default: `true`

:   Whether to fail action task if a single waypoint fails. If false, will continue to next waypoint.

### **`loop_rate`**

Type: `int` Default: `20`

:   Rate to check for results from current navigation task.

### **`global_frame_id`**

Type: `string` Default: `"map"`

:   The name of the global coordinate frame published by robot_localization. Only used by the gps_waypoint_follower to
    convert GPS waypoints to this frame.

### **`introspection_mode`**

Type: `string` Default: `"disabled"`

:   The introspection mode for services and actions. Options are "disabled", "metadata", "contents".

### **`bond_heartbeat_period`**

Type: `double` Default: `0.25`

:   The lifecycle node bond mechanism publishing period (on the /bond topic). Disabled if inferior or equal to 0.0.

### **`allow_parameter_qos_overrides`**

Type: `bool` Default: `true`

:   Whether to allow QoS profiles to be overwritten with parameterized values.

### **`waypoint_task_executor_plugin`**

Type: `string` Default: `"wait_at_waypoint"`

:   A plugin to define tasks to be executed when robot arrives to a waypoint.

    Note
    :   The plugin namespace defined needs to have a `plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:
        ```yaml
        waypoint_follower:
          ros__parameters:
            waypoint_task_executor_plugin: "wait_at_waypoint"
            wait_at_waypoint:
              plugin: "nav2_waypoint_follower::WaitAtWaypoint"
              enabled: True
              waypoint_pause_duration: 0
        ```

## Provided Plugins

The plugins listed below are inside the `nav2_waypoint_follower` namespace.

<div class="grid" markdown>

[WaitAtWaypoint][wait-at-waypoint]{ .md-button .md-button--primary }
[PhotoAtWaypoint][photo-at-waypoint]{ .md-button .md-button--primary }
[InputAtWaypoint][input-at-waypoint]{ .md-button .md-button--primary }

</div>

## Default Plugin

| Namespace          | Plugin                                   |
|--------------------|------------------------------------------|
| "wait_at_waypoint" | "nav2_waypoint_follower::WaitAtWaypoint" |

## Example

```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    introspection_mode: "disabled"
      wait_at_waypoint:
        plugin: "nav2_waypoint_follower::WaitAtWaypoint"
        enabled: True
        waypoint_pause_duration: 0
```
