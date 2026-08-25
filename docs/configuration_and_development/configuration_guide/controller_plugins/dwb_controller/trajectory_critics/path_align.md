# PathAlignCritic { #path-align-critic }

Scores a trajectory based on how well it is aligned to the path provided by the global planner.

## Parameters

`<name>`: `PathAlignCritic` critic name defined in the **`<dwb plugin>.critics`** parameter defined in [DWB Controller][dwb-controller].

### **`<dwb plugin>.<name>.forward_point_distance`**

Type: `double` Default: `0.325`

:   Point in front of robot to look ahead to compute angular change from.

### **`<dwb plugin>.<name>.aggregation_type`**

Type: `string` Default: `"last"`

:   last, sum, or product combination methods.

### **`<dwb plugin>.<name>.scale`**

Type: `double` Default: `1.0`

:   Weighed scale for critic.
