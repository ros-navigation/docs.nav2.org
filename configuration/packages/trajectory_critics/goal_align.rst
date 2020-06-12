.. _configuring_dwb_goal_align:

GoalAlignCritic
===============

Scores a trajectory based on how well aligned the trajectory is with the goal pose.

Parameters
**********

``<dwb plugin>``: DWB plugin name defined in the **controller_plugin_ids** parameter in :ref:`configuring_controller_server`.

``<name>``: GoalAlignCritic critic name defined in the **<dwb plugin>.critics** parameter defined in :ref:`dwb_controller`.

:``<dwb plugin>``.\ ``<name>``.forward_point_distance:

  ====== =======
  Type   Default
  ------ -------
  double 0.325 
  ====== =======
    
    Description
        Point in front of robot to look ahead to compute angular change from.

:``<dwb plugin>``.\ ``<name>``.aggregation_type:

  ====== =======
  Type   Default
  ------ -------
  string "last" 
  ====== =======
    
    Description
        last, sum, or product combination methods.

:``<dwb plugin>``.\ ``<name>``.scale:

  ====== =======
  Type   Default
  ------ -------
  double 1.0 
  ====== =======
    
    Description
        Weighed scale for critic.
