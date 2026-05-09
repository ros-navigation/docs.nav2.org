.. _configuring_nav2_controller_adaptive_tolerance_goal_checker_plugin:

AdaptiveToleranceGoalChecker
============================

The adaptive tolerance goal checker uses two underlying goal tolerances a fine and a coarse one. The fine tolerance is used to instantly trigger goal reached when the robot is close to the goal (functionally same as simple goal checker), while the coarse tolerance is used to trigger goal reached when the robot is further from the goal but is making no meaningful progress towards it (or not expected to).

The goal is cosidered reached when one of the following conditions is met:
  - The robot is within the fine goal tolerance
  - The robot is within the coarse goal tolerance and its linear velocity and orientational velocity are below a specified threshold for a set amount of cycles
  - The robot is within the coarse goal tolerance and robots distance to the goal is not improving for a set amount of cycles
  - The robot is within the coarse goal tolerance and it has passed the finish line (the line perpendicular to the first robot pose within the coarse tolerance and passing through the goal pose)

.. image:: /images/adaptive_tolerance_goal_checker.png
   :alt: AdaptiveToleranceGoalChecker Illustration
   :align: center

Parameters
**********

``<nav2_controller plugin>``: nav2_controller plugin name defined in the **goal_checker_plugin_id** parameter in :ref:`configuring_controller_server`.

:``<nav2_controller plugin>``.fine_xy_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.10
  ====== =======

    Description
        Fine (desired) XY tolerance to the goal (m). When the robot's XY distance to the goal is within this tolerance, the goal is considered reached immediately (subject to the yaw check). Should be smaller than ``coarse_xy_goal_tolerance``.

:``<nav2_controller plugin>``.coarse_xy_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======

    Description
        Coarse (fallback) XY tolerance to the goal (m). When the robot is within this tolerance but outside the fine tolerance, the goal is considered reached only if one of the coarse-tier acceptance conditions fires (stopped stagnation, distance stagnation, or finish-line crossing). Should be larger than ``fine_xy_goal_tolerance``.

:``<nav2_controller plugin>``.yaw_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======

    Description
        Tolerance on the yaw angle to the goal orientation (rad). The goal is only considered reached once the XY check passes AND the yaw error is within this tolerance.

:``<nav2_controller plugin>``.path_length_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 1.0
  ====== =======

    Description
        Maximum remaining transformed global plan length above which the goal check is skipped (m). Prevents premature goal acceptance when the robot is still far from the goal along the path.

:``<nav2_controller plugin>``.stateful:

  ==== =======
  Type Default
  ---- -------
  bool true
  ==== =======

    Description
        If true, once the XY tolerance is satisfied the checker latches that result and only re-evaluates yaw on subsequent cycles. If false, both XY and yaw are re-checked every cycle.

:``<nav2_controller plugin>``.symmetric_yaw_tolerance:

  ==== =======
  Type Default
  ---- -------
  bool false
  ==== =======

    Description
        If true, the yaw check accepts both the goal orientation and its 180° opposite (useful for symmetric robots that can drive forward or backward). If false, only the goal orientation is accepted.

:``<nav2_controller plugin>``.trans_stopped_velocity:

  ====== =======
  Type   Default
  ------ -------
  double 0.10
  ====== =======

    Description
        Linear velocity threshold below which the robot is considered "stopped" for the stopped-stagnation acceptance path (m/s). Combined with ``rot_stopped_velocity`` and ``required_stagnation_cycles``.

:``<nav2_controller plugin>``.rot_stopped_velocity:

  ====== =======
  Type   Default
  ------ -------
  double 0.10
  ====== =======

    Description
        Angular velocity threshold below which the robot is considered "stopped" for the stopped-stagnation acceptance path (rad/s). Combined with ``trans_stopped_velocity`` and ``required_stagnation_cycles``.

:``<nav2_controller plugin>``.required_stagnation_cycles:

  ==== =======
  Type Default
  ---- -------
  int  15
  ==== =======

    Description
        Number of consecutive controller cycles for which a stagnation condition must hold before the coarse-tier acceptance fires. Applies to both stopped stagnation (velocity below threshold) and distance stagnation (no improvement in distance to goal). Must be ``>= 1``.

Example
*******

.. code-block:: yaml

    controller_server:
      ros__parameters:
        goal_checker_plugins: ["goal_checker"]
        goal_checker:
          plugin: "nav2_controller::AdaptiveToleranceGoalChecker"
          fine_xy_goal_tolerance: 0.10
          coarse_xy_goal_tolerance: 0.25
          yaw_goal_tolerance: 0.25
          path_length_tolerance: 1.0
          stateful: true
          symmetric_yaw_tolerance: false
          trans_stopped_velocity: 0.10
          rot_stopped_velocity: 0.10
          required_stagnation_cycles: 15
