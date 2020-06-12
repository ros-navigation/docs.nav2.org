.. _configuring_dwb_obstacle_footprint:

ObstacleFootprintCritic
=======================

Scores a trajectory based on verifying all points along the robot's footprint don't touch an obstacle marked in the costmap.

Parameters
**********

``<dwb plugin>``: DWB plugin name defined in the **controller_plugin_ids** parameter in :ref:`configuring_controller_server`.

``<name>``: ObstacleFootprintCritic critic name defined in the **<dwb plugin>.critics** parameter defined in :ref:`dwb_controller`.


:``<dwb plugin>``.\ ``<name>``.sum_scores:

  ==== =======
  Type Default
  ---- -------
  bool false 
  ==== =======
    
    Description
        Whether to allow for scores to be summed up.

:``<dwb plugin>``.\ ``<name>``.scale:

  ====== =======
  Type   Default
  ------ -------
  double 1.0 
  ====== =======
    
    Description
        Weighed scale for critic.
