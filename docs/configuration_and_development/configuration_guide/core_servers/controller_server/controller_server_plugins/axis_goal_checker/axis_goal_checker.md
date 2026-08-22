# AxisGoalChecker { #axis-goal-checker }

Checks whether the robot has reached the goal pose by projecting the robot's position onto the path direction near the goal. This goal checker estimates the path direction from the `goal_pose` and the first plan pose at least `direction_estimation_distance` behind it, then checks if the robot is within tolerance along that axis.

Unlike simple distance-based goal checkers, the AxisGoalChecker allows independent control of tolerances along the path direction (`along_path_tolerance`) and perpendicular to it (`cross_track_tolerance`). This is particularly useful for applications where precise alignment along a specific axis is more important than radial distance from the goal.

<figure markdown="span">
  ![AxisGoalChecker Illustration](assets/axis_goal_checker.png){ title="AxisGoalChecker Illustration" }
</figure>

## Parameters

`<nav2_controller plugin>`: nav2_controller plugin name defined in the **`goal_checker_plugin_id`** parameter in [Controller Server][controller-server].

### **`<nav2_controller plugin>.along_path_tolerance`**

Type: `double` Default: `0.25`

:   Tolerance for the projected distance along the path direction (m). This checks how far ahead or behind the goal the robot is when projected onto the path axis.

### **`<nav2_controller plugin>.cross_track_tolerance`**

Type: `double` Default: `0.25`

:   Tolerance for the perpendicular distance from the path direction (m). This checks how far to the left or right of the path axis the robot is.

### **`<nav2_controller plugin>.path_length_tolerance`**

Type: `double` Default: `1.0`

:   Maximum path length to consider for goal checking (m). If the remaining path length exceeds this value, the goal check is skipped. This prevents premature goal acceptance when far from the goal.

### **`<nav2_controller plugin>.direction_estimation_distance`**

Type: `double` Default: `0.15`

:   Distance back along the plan, from the goal, used to estimate the approach direction (m). The checker walks back from the goal and uses the first pose at least this far away as the direction reference. Larger values are more robust to erroneous end-of-plan poses and sharp final turns, but average the direction over a longer segment. Must be greater than `0` and less than `path_length_tolerance`.

### **`<nav2_controller plugin>.is_overshoot_valid`**

Type: `bool` Default: `false`

:   Whether to allow overshooting past the goal along the path direction. When `false` (default), uses `fabs(projected_distance) < along_path_tolerance` for symmetric tolerance. When `true`, uses `projected_distance < along_path_tolerance`, allowing the robot to be any distance past the goal but still requiring it to be within tolerance if before the goal.
