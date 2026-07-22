# ObstacleFootprintCritic { #obstacle-footprint-critic }

Scores a trajectory based on verifying all points along the robot's footprint don't touch an obstacle marked in the costmap.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **`controller_plugin_ids`** parameter in [Controller Server][controller-server].

`<name>`: `ObstacleFootprintCritic` critic name defined in the **`<dwb plugin>.critics`** parameter defined in [DWB Controller][dwb-controller].

### **`<dwb plugin>.<name>.sum_scores`**

Type: `bool` Default: `false`

:   Whether to allow for scores to be summed up.

### **`<dwb plugin>.<name>.scale`**

Type: `double` Default: `1.0`

:   Weighed scale for critic.
