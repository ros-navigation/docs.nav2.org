# PositionGoalChecker { #position-goal-checker }

Checks whether the robot has reached the goal pose.

## Parameters

`<nav2_controller plugin>`: nav2_controller plugin name defined in the **`goal_checker_plugin_id`** parameter in [Controller Server][controller-server].

### **`<nav2_controller plugin>.xy_goal_tolerance`**

Type: `double` Default: `0.25`

:   Tolerance to meet goal completion criteria (m).

### **`<nav2_controller plugin>.xy_goal_tolerance_buffer`**

Type: `double` Default: `0.0`

:   Hysteresis buffer for stateful XY position checking (m). When `stateful` is `true`, after the XY goal condition has been satisfied, the robot may drift within `xy_goal_tolerance + xy_goal_tolerance_buffer` without rechecking the XY position. If the robot moves outside this buffered region, the previous XY reached state is cleared and the XY position must be checked again.

### **`<nav2_controller plugin>.path_length_tolerance`**

Type: `double` Default: `1.0`

:   Tolerance to meet goal completion criteria (m).

### **`<nav2_controller plugin>.stateful`**

Type: `bool` Default: `true`

:   Whether to check for XY position tolerance after rotating to goal orientation in case of minor localization changes.
