.. _configuring_nav2_controller_stopped_goal_checker_plugin:

StoppedGoalChecker
==================

Checks whether the robot has reached the goal pose and come to a stop.

Parameters
**********

``<nav2_controller plugin>``: nav2_controller plugin name defined in the **goal_checker_plugin_id** parameter in :ref:`configuring_controller_server`.

:``<nav2_controller plugin>``.trans_stopped_velocity:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======
    
    Description
        Velocity below is considered to be stopped at tolerance met (m/s).

:``<nav2_controller plugin>``.rot_stopped_velocity:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======
    
    Description
        Velocity below is considered to be stopped at tolerance met (rad/s).

:``<nav2_controller plugin>``.xy_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======
    
    Description
        Tolerance to meet goal completion criteria (m).

:``<nav2_controller plugin>``.yaw_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======
    
    Description
        Tolerance to meet goal completion criteria (rad).

:``<nav2_controller plugin>``.stateful:

  ==== =======
  Type Default
  ---- -------
  bool true 
  ==== =======
    
    Description
        Whether to check for XY position tolerance after rotating to goal orientation in case of minor localization changes.
