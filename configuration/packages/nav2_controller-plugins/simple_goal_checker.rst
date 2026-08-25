.. _configuring_nav2_controller_simple_goal_checker_plugin:

SimpleGoalChecker
=================

Checks whether the robot has reached the goal pose.

Parameters
**********

``<nav2_controller plugin>``: nav2_controller plugin name defined in the **goal_checker_plugin_id** parameter in :ref:`configuring_controller_server`.

:``<nav2_controller plugin>``.xy_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======

    Description
        Tolerance to meet goal completion criteria (m).

:``<nav2_controller plugin>``.xy_goal_tolerance_buffer:

  ====== =======
  Type   Default
  ------ -------
  double 0.0
  ====== =======

    Description
        Hysteresis buffer for stateful XY position checking (m). When ``stateful`` is true, after the XY goal condition has been satisfied, the robot may drift within ``xy_goal_tolerance + xy_goal_tolerance_buffer`` without rechecking the XY position. If the robot moves outside this buffered region, the previous XY reached state is cleared and the XY position must be checked again.


:``<nav2_controller plugin>``.yaw_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======

    Description
        Tolerance to meet goal completion criteria (rad).

:``<nav2_controller plugin>``.path_length_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 1.0
  ====== =======

    Description
        Tolerance to meet goal completion criteria (m).

:``<nav2_controller plugin>``.stateful:

  ==== =======
  Type Default
  ---- -------
  bool true
  ==== =======

    Description
        Whether to check for XY position tolerance after rotating to goal orientation in case of minor localization changes.

:``<nav2_controller plugin>``.symmetric_yaw_tolerance:

  ==== =======
  Type Default
  ---- -------
  bool false
  ==== =======

    Description
        Enable symmetric goal orientation acceptance. When enabled, the robot accepts the goal as reached when oriented at either the goal orientation or the goal orientation + 180°. This is useful for symmetric robots (e.g., differential drives with sensors on both ends) that can navigate equally well in forward and backward directions and does not care which direction it ends in (i.e. controller algorithm decides). See :ref:`tuning` for detailed information.
