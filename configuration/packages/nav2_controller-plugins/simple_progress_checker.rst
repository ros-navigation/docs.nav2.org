.. _configuring_nav2_controller_simple_progress_checker_plugin:

SimpleProgressChecker
=====================

Checks whether the robot has made progress.

Parameters
**********

``<nav2_controller plugin>``: nav2_controller plugin name defined in the **progress_checker_plugin_id** parameter in :ref:`configuring_controller_server`.

:``<nav2_controller plugin>``.required_movement_radius:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.5
  ============== =============================

  Description
    Minimum amount a robot must move to be progressing to goal (m).

:``<nav2_controller plugin>``.movement_time_allowance:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         10.0
  ============== =============================

  Description
    Maximum amount of time a robot has to move the minimum radius (s).

