<a id="configuring-nav2-controller-simple-progress-checker-plugin"></a>

# SimpleProgressChecker

Checks whether the robot has made positional progress.

## Parameters

`<nav2_controller plugin>`: nav2_controller plugin name defined in the **progress_checker_plugin_id** parameter in [Controller Server](../configuring-controller-server.md#configuring-controller-server).

* **`<nav2_controller plugin>`.required_movement_radius:**
  | Type   |   Default |
  |--------|-----------|
  | double |       0.5 |

  Description
  : Minimum amount a robot must move to be progressing to goal (m).
* **`<nav2_controller plugin>`.movement_time_allowance:**
  | Type   |   Default |
  |--------|-----------|
  | double |        10 |

  Description
  : Maximum amount of time a robot has to move the minimum radius (s).

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
