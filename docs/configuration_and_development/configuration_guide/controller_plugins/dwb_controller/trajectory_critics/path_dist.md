# PathDistCritic { #path-dist-critic }

Scores a trajectory based on how well it is aligned to the path provided by the global planner.

## Parameters

`<name>`: PathDistCritic critic name defined in the **`<dwb plugin>.critics`** parameter defined in [DWB Controller][dwb-controller].

### **`<dwb plugin>.<name>.aggregation_type`**

Type: `string` Default: `"last"`

:   last, sum, or product combination methods.

### **`<dwb plugin>.<name>.scale`**

Type: `double` Default: `1.0`

:   Weighed scale for critic.
