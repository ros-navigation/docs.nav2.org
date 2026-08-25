# RotateToGoalCritic { #rotate-to-goal-critic }

Only allows the robot to rotate to the goal orientation when it is sufficiently close to the goal location.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **`controller_plugin_ids`** parameter in [Controller Server][controller-server].

`<name>`: `RotateToGoalCritic` critic name defined in the **`<dwb plugin>.critics`** parameter defined in [DWB Controller][dwb-controller].

### **`<dwb plugin>.xy_goal_tolerance`**

Type: `double` Default: `0.25`

:   Tolerance to meet goal completion criteria (m).

### **`<dwb plugin>.path_length_tolerance`**

Type: `double` Default: `1.0`

:   Tolerance to meet goal completion criteria (m).

### **`<dwb plugin>.trans_stopped_velocity`**

Type: `double` Default: `0.25`

:   Velocity below is considered to be stopped at tolerance met (rad/s).

### **`<dwb plugin>.<name>.slowing_factor`**

Type: `double` Default: `5.0`

:   Factor to slow robot motion by while rotating to goal.

### **`<dwb plugin>.<name>.lookahead_time`**

Type: `double` Default: `-1.0`

:   If `> 0`, amount of time to look forward for a collision for.

### **`<dwb plugin>.<name>.scale`**

Type: `double` Default: `1.0`

:   Weighed scale for critic.
