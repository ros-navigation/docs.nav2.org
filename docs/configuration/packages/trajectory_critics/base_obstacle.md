# BaseObstacleCritic

Scores a trajectory based on where the path passes over the costmap.
To use this properly, you must use the inflation layer in costmap to expand obstacles by the robot’s radius.

## Parameters

`<dwb plugin>`: DWB plugin name defined in the **controller_plugin_ids** parameter in [Controller Server](../configuring-controller-server.md#controller-server).

`<name>`: BaseObstacleCritic critic name defined in the **<dwb plugin>.critics** parameter defined in [DWB Controller](../dwb-params/controller.md#dwb-controller).

* **`<dwb plugin>`.`<name>`.sum_scores:**
  | Type   | Default   |
  |--------|-----------|
  | bool   | false     |
    Description
    : Whether to allow for scores to be summed up.
* **`<dwb plugin>`.`<name>`.scale:**
  | Type   |   Default |
  |--------|-----------|
  | double |         1 |
    Description
    : Weighed scale for critic.
