# Publisher { #publisher }

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **controller_plugin_ids** parameter in [Controller Server][controller-server].

### **`<dwb plugin>.publish_evaluation`**

Type: `bool` Default: `true`

:   Whether to publish the local plan evaluation.

### **`<dwb plugin>.publish_local_plan`**

Type: `bool` Default: `true`

:   Whether to publish the local planner's plan.

### **`<dwb plugin>.publish_trajectories`**

Type: `bool` Default: `true`

:   Whether to publish debug trajectories.

### **`<dwb plugin>.publish_cost_grid_pc`**

Type: `bool` Default: `false`

:   Whether to publish the cost grid.

### **`<dwb plugin>.marker_lifetime`**

Type: `double` Default: `0.1`

:   How long for the marker to remain.
