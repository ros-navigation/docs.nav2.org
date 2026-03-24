# SimpleProgressChecker { #simple-progress-checker }

Checks whether the robot has made positional progress.

## Parameters

`<nav2_controller plugin>`: nav2_controller plugin name defined in the **progress_checker_plugin_id** parameter in [Controller Server](../configuring-controller-server.md#controller-server).

### **`<nav2_controller plugin>.required_movement_radius`**

| Type     | Default |
|----------|---------|
| `double` | 0.5     |

Description
:   Minimum amount a robot must move to be progressing to goal (m).

### **`<nav2_controller plugin>.movement_time_allowance`**

| Type     | Default |
|----------|---------|
| `double` | 10.0    |

Description
:   Maximum amount of time a robot has to move the minimum radius (s).
