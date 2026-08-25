# PoseProgressChecker { #pose-progress-checker }

Checks whether the robot has made progress based on translation and rotation.

## Parameters

`<nav2_controller plugin>`: nav2_controller plugin name defined in the **`progress_checker_plugin_id`** parameter in [Controller Server][controller-server].

### **`<nav2_controller plugin>.required_movement_radius`**

Type: `double` Default: `0.5`

:   Minimum amount a robot must move to be progressing to goal (m).

### **`<nav2_controller plugin>.required_movement_angle`**

Type: `double` Default: `0.5`

:   Minimum amount a robot must rotate to be progressing to goal (rad).

### **`<nav2_controller plugin>.movement_time_allowance`**

Type: `double` Default: `10.0`

:   Maximum amount of time a robot has to move the minimum radius or the mnimum angle (s).
