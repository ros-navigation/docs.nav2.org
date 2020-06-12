.. _configuring_dwb_simple_goal_checker_plugin:

SimpleGoalChecker
=================

Checks whether the robot has reached the goal pose.

Parameters
**********

``<dwb plugin>``: DWB plugin name defined in the **controller_plugin_ids** parameter in :ref:`configuring_controller_server`.

:``<dwb plugin>``.xy_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======
    
    Description
        Tolerance to meet goal completion criteria (m).

:``<dwb plugin>``.yaw_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======
    
    Description
        Tolerance to meet goal completion criteria (rad).

:``<dwb plugin>``.stateful:

  ==== =======
  Type Default
  ---- -------
  bool true 
  ==== =======
    
    Description
        Whether to check for XY position tolerance after rotating to goal orientation in case of minor localization changes.
