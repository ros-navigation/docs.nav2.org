# DWB Controller { #dwb-controller }

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **`controller_plugin_ids`** parameter in [Controller Server][controller-server].

### **`<dwb plugin>.critics`**

Type: `vector<string>` Default: `N/A`

:   List of critic plugins to use.

### **`<dwb plugin>.default_critic_namespaces`**

Type: `vector<string>` Default: `["dwb_critics"]`

:   Namespaces to load critics in.

### **`<dwb plugin>.debug_trajectory_details`**

Type: `bool` Default: `false`

:   Publish debug information (on what topic???).

### **`<dwb plugin>.trajectory_generator_name`**

Type: `string` Default: `"dwb_plugins::StandardTrajectoryGenerator"`

:   Trajectory generator plugin name.

### **`<dwb plugin>.goal_checker_name`**

Type: `string` Default: `"dwb_plugins::SimpleGoalChecker"`

:   Goal checker plugin name.

### **`<dwb plugin>.short_circuit_trajectory_evaluation`**

Type: `bool` Default: `true`

:   Stop evaluating scores after best score is found.

### **`<dwb plugin>.path_distance_bias`** (Legacy)

Type: `double` Default: `N/A`

:   Old version of `PathAlign.scale`, use that instead.

### **`<dwb plugin>.goal_distance_bias`** (Legacy)

Type: `double` Default: `N/A`

:   Old version of `GoalAlign.scale`, use that instead.

### **`<dwb plugin>.occdist_scale`** (Legacy)

Type: `double` Default: `N/A`

:   Old version of `ObstacleFootprint.scale`, use that instead.

### **`<dwb plugin>.max_scaling_factor`** (Legacy)

Type: `double` Default: `N/A`

:   Old version of `ObstacleFootprint.max_scaling_factor`, use that instead.

### **`<dwb plugin>.scaling_speed`** (Legacy)

Type: `double` Default: `N/A`

:   Old version of `ObstacleFootprint.scaling_speed`, use that instead.

### **`<dwb plugin>.PathAlign.scale`**

Type: `double` Default: `32.0`

:   Scale for path align critic, overriding local default.

### **`<dwb plugin>.GoalAlign.scale`**

Type: `double` Default: `24.0`

:   Scale for goal align critic, overriding local default.

### **`<dwb plugin>.PathDist.scale`**

Type: `double` Default: `32.0`

:   Scale for path distance critic, overriding local default.

### **`<dwb plugin>.GoalDist.scale`**

Type: `double` Default: `24.0`

:   Scale for goal distance critic, overriding local default.
