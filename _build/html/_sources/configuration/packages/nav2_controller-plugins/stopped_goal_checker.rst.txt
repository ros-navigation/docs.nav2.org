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
