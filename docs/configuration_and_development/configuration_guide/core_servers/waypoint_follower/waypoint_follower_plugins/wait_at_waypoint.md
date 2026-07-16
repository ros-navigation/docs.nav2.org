# WaitAtWaypoint { #wait-at-waypoint }

Lets robot to pause for a specified amount of time after reaching each waypoints.

## Parameters

`<nav2_waypoint_follower plugin>`: nav2_waypoint_follower plugin name defined in the **waypoint_task_executor_plugin_id** parameter in [Waypoint Follower][waypoint-follower].

### **`<nav2_waypoint_follower plugin>.enabled`**

Type: `bool` Default: `true`

:   Whether waypoint_task_executor plugin is enabled.

### **`<nav2_waypoint_follower plugin>.waypoint_pause_duration`**

Type: `int` Default: `0`

:   Amount of time in milliseconds, for robot to sleep/wait after each waypoint is reached. If zero, robot will directly continue to next waypoint.
