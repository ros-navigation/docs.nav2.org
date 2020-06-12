.. _configuring_dwb_stopped_goal_checker_plugin:

StoppedGoalChecker
==================

Checks whether the robot has reached the goal pose and come to a stop.

Parameters
**********

``<dwb plugin>``: DWB plugin name defined in the **controller_plugin_ids** parameter in :ref:`configuring_controller_server`.

:``<dwb plugin>``.trans_stopped_velocity:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======
    
    Description
        Velocity below is considered to be stopped at tolerance met (m/s).

:``<dwb plugin>``.rot_stopped_velocity:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======
    
    Description
        Velocity below is considered to be stopped at tolerance met (rad/s).
