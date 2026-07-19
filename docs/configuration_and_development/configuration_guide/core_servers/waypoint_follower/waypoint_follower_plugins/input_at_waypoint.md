# InputAtWaypoint { #input-at-waypoint }

Lets robot to wait for external input, with timeout, at a waypoint.

## Parameters

`<nav2_waypoint_follower plugin>`: `nav2_waypoint_follower` plugin name defined in the **`waypoint_task_executor_plugin_id`** parameter in [Waypoint Follower][waypoint-follower].

### **`<nav2_waypoint_follower plugin>.enabled`**

Type: `bool` Default: `true`

:   Whether `waypoint_task_executor` plugin is enabled.

### **`<nav2_waypoint_follower plugin>.timeout`**

Type: `double` Default: `10.0`

:   Amount of time in seconds to wait for user input before moving on to the next waypoint.

### **`<nav2_waypoint_follower plugin>.input_topic`**

Type: `string` Default: `"input_at_waypoint/input"`

:   Topic input is published to to indicate to move to the next waypoint, in `std_msgs/Empty`.

### **`allow_parameter_qos_overrides`**

Type: `bool` Default: `true`

:   Whether to allow QoS profiles to be overwritten with parameterized values.
