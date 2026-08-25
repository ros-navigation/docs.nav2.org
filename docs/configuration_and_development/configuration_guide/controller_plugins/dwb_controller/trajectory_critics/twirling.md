# TwirlingCritic { #twirling-critic }

Prevents holonomic robots from spinning as they make their way to the goal.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **`controller_plugin_ids`** parameter in [Controller Server][controller-server].

`<name>`: `TwirlingCritic` critic name defined in the **`<dwb plugin>.critics`** parameter defined in [DWB Controller][dwb-controller].

### **`<dwb plugin>.<name>.scale`**

Type: `double` Default: `1.0`

:   Weighed scale for critic.
