# TwirlingCritic

Prevents holonomic robots from spinning as they make their way to the goal.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **controller_plugin_ids** parameter in [Controller Server](../configuring-controller-server.md#controller-server).

`<name>`: TwirlingCritic critic name defined in the **<dwb plugin>.critics** parameter defined in [DWB Controller](../dwb-params/controller.md#dwb-controller).

### **`<dwb plugin>.<name>.scale`**

| Type   |   Default |
|--------|-----------|
| double |       1.0 |

Description
:   Weighed scale for critic.
