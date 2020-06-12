.. _configuring_dwb_goal_dist:

GoalDistCritic
==============

Scores a trajectory based on how close the trajectory gets the robot to the goal pose.

Parameters
**********

``<dwb plugin>``: DWB plugin name defined in the **controller_plugin_ids** parameter in :ref:`configuring_controller_server`.

``<name>``: GoalDistCritic critic name defined in the **<dwb plugin>.critics** parameter defined in :ref:`dwb_controller`.


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
