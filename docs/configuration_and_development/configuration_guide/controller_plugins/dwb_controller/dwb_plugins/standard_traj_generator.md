# StandardTrajectoryGenerator { #standard-trajectory-generator }

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **`controller_plugin_ids`** parameter in [Controller Server][controller-server].

### **`<dwb plugin>.sim_time`**

Type: `double` Default: `1.7`

:   Time to simulate ahead by (s).

### **`<dwb plugin>.discretize_by_time`**

Type: `bool` Default: `false`

:   If `true`, forward simulate by time. If `false`, forward simulate by linear and angular granularity.

### **`<dwb plugin>.time_granularity`**

Type: `double` Default: `0.5`

:   Time ahead to project.

### **`<dwb plugin>.linear_granularity`**

Type: `double` Default: `0.5`

:   Linear distance forward to project.

### **`<dwb plugin>.angular_granularity`**

Type: `double` Default: `0.025`

:   Angular distance to project.

### **`<dwb plugin>.include_last_point`**

Type: `bool` Default: `true`

:   Whether to include the last pose in the trajectory.

### **`<dwb plugin>.limit_vel_cmd_in_traj`**

Type: `bool` Default: `false`

:   Whether to limit velocity command in trajectory using sampled velocity instead of the commanded velocity.
