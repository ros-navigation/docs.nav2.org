# PreferForwardCritic { #prefer-forward-critic }

Scores trajectories that move the robot forwards more highly.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **`controller_plugin_ids`** parameter in [Controller Server][controller-server].

`<name>`: PreferForwardCritic critic name defined in the **`<dwb plugin>.critics`** parameter defined in [DWB Controller][dwb-controller].

### **`<dwb plugin>.<name>.penalty`**

Type: `double` Default: `1.0`

:   Penalty to apply to backward motion.

### **`<dwb plugin>.<name>.strafe_x`**

Type: `double` Default: `0.1`

:   Minimum X velocity before penalty.

### **`<dwb plugin>.<name>.strafe_theta`**

Type: `double` Default: `0.2`

:   Minimum angular velocity before applying penalty.

### **`<dwb plugin>.<name>.theta_scale`**

Type: `double` Default: `10.0`

:   Weight for angular velocity component.

### **`<dwb plugin>.<name>.scale`**

Type: `double` Default: `1.0`

:   Weighed scale for critic.
