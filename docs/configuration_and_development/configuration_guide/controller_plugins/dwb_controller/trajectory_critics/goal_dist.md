# GoalDistCritic { #goal-dist-critic }

Scores a trajectory based on how close the trajectory gets the robot to the goal pose.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **`controller_plugin_ids`** parameter in [Controller Server][controller-server].

`<name>`: `GoalDistCritic` critic name defined in the **`<dwb plugin>.critics`** parameter defined in [DWB Controller][dwb-controller].

### **`<dwb plugin>.<name>.aggregation_type`**

Type: `string` Default: `"last"`

:   last, sum, or product combination methods.

### **`<dwb plugin>.<name>.obstacle_aware_propagation`**

Type: `bool` Default: `false`

:   Whether to measure the distance to the goal around lethal obstacles instead of through them. Falls back to straight line distances if the robot cannot be reached around obstacles.

### **`<dwb plugin>.<name>.scale`**

Type: `double` Default: `1.0`

:   Weighed scale for critic.
