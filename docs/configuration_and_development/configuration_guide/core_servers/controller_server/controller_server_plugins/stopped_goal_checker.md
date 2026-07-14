# StoppedGoalChecker { #stopped-goal-checker }

Checks whether the robot has reached the goal pose and come to a stop.

## Parameters

`<nav2_controller plugin>`: nav2_controller plugin name defined in the **goal_checker_plugin_id** parameter in [Controller Server][controller-server].

### **`<nav2_controller plugin>.trans_stopped_velocity`**

Type: `double` Default: `0.25`

:   Velocity below is considered to be stopped at tolerance met (m/s).

### **`<nav2_controller plugin>.rot_stopped_velocity`**

Type: `double` Default: `0.25`

:   Velocity below is considered to be stopped at tolerance met (rad/s).

### **`<nav2_controller plugin>.xy_goal_tolerance`**

Type: `double` Default: `0.25`

:   Tolerance to meet goal completion criteria (m).

### **`<nav2_controller plugin>.yaw_goal_tolerance`**

Type: `double` Default: `0.25`

:   Tolerance to meet goal completion criteria (rad).

### **`<nav2_controller plugin>.path_length_tolerance`**

Type: `double` Default: `1.0`

:   Tolerance to meet goal completion criteria (m).

### **`<nav2_controller plugin>.stateful`**

Type: `bool` Default: `true`

:   Whether to check for XY position tolerance after rotating to goal orientation in case of minor localization changes.

### **`<nav2_controller plugin>.symmetric_yaw_tolerance`**

Type: `bool` Default: `false`

:   Enable symmetric goal orientation acceptance. When enabled, the robot accepts the goal as reached when oriented at either the goal orientation or the goal orientation + 180°. This is useful for symmetric robots (e.g., differential drives with sensors on both ends) that can navigate equally well in forward and backward directions and does not care which direction it ends in (i.e. controller algorithm decides). See [Tuning Guide][tuning-guide] for detailed information.
