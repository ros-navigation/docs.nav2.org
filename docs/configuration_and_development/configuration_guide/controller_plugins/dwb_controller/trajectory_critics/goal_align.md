# GoalAlignCritic { #goal-align-critic }

Scores a trajectory based on how well aligned the trajectory is with the goal pose.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **`controller_plugin_ids`** parameter in [Controller Server][controller-server].

`<name>`: `GoalAlignCritic` critic name defined in the **`<dwb plugin>.critics`** parameter defined in [DWB Controller][dwb-controller].

### **`<dwb plugin>.<name>.forward_point_distance`**

Type: `double` Default: `0.325`

:   Point in front of robot to look ahead to compute angular change from.

### **`<dwb plugin>.<name>.aggregation_type`**

Type: `string` Default: `"last"`

:   last, sum, or product combination methods.

### **`<dwb plugin>.<name>.scale`**

Type: `double` Default: `1.0`

:   Weighed scale for critic.
