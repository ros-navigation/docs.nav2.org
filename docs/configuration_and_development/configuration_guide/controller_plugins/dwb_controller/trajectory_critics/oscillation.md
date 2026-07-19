# OscillationCritic { #oscillation-critic }

Prevents the robot from just moving backwards and forwards.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **`controller_plugin_ids`** parameter in [Controller Server][controller-server].

`<name>`: OscillationCritic critic name defined in the **`<dwb plugin>.critics`** parameter defined in [DWB Controller][dwb-controller].

### **`<dwb plugin>.<name>.oscillation_reset_dist`**

Type: `double` Default: `0.05`

:   Minimum distance to move to reset oscillation watchdog (m).

### **`<dwb plugin>.<name>.oscillation_reset_angle`**

Type: `double` Default: `0.2`

:   Minimum angular distance to move to reset watchdog (rad).

### **`<dwb plugin>.<name>.oscillation_reset_time`**

Type: `double` Default: `-1.0`

:   Duration when a reset may be called. If -1, cannot be reset..

### **`<dwb plugin>.<name>.x_only_threshold`**

Type: `double` Default: `0.05`

:   Threshold to check in the X velocity direction.

### **`<dwb plugin>.<name>.scale`**

Type: `double` Default: `1.0`

:   Weighed scale for critic.
