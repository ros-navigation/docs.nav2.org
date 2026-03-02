<a id="configuring-dwb-obstacle-footprint"></a>

# ObstacleFootprintCritic

Scores a trajectory based on verifying all points along the robot’s footprint don’t touch an obstacle marked in the costmap.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **controller_plugin_ids** parameter in [Controller Server](../configuring-controller-server.md#configuring-controller-server).

`<name>`: ObstacleFootprintCritic critic name defined in the **<dwb plugin>.critics** parameter defined in [DWB Controller](../dwb-params/controller.md#dwb-controller).

* **`<dwb plugin>`.`<name>`.sum_scores:**
  | Type   | Default   |
  |--------|-----------|
  | bool   | false     |
  > Description
  > : Whether to allow for scores to be summed up.
* **`<dwb plugin>`.`<name>`.scale:**
  | Type   |   Default |
  |--------|-----------|
  | double |         1 |
  > Description
  > : Weighed scale for critic.

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
