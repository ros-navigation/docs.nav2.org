# GoalDistCritic { #goal-dist-critic }

Scores a trajectory based on how close the trajectory gets the robot to the goal pose.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **controller_plugin_ids** parameter in [Controller Server](../configuring-controller-server.md#controller-server).

`<name>`: GoalDistCritic critic name defined in the **<dwb plugin>.critics** parameter defined in [DWB Controller](../dwb-params/controller.md#dwb-controller).

### **`<dwb plugin>.<name>.aggregation_type`**

| Type     | Default |
|----------|---------|
| `string` | “last”  |

Description
:   last, sum, or product combination methods.

### **`<dwb plugin>.<name>.scale`**

| Type     | Default |
|----------|---------|
| `double` | 1.0     |

Description
:   Weighed scale for critic.
