.. _configuring_nav2_controller_axis_goal_checker_plugin:

AxisGoalChecker
===============

Checks whether the robot has reached the goal pose by projecting the robot's position onto the path direction defined by the last segment of the path. This goal checker uses the last two poses of the path (``before_goal_pose`` and ``goal_pose``) to determine the path direction and then checks if the robot is within tolerance along that axis.

Unlike simple distance-based goal checkers, the AxisGoalChecker allows independent control of tolerances along the path direction (``along_path_tolerance``) and perpendicular to it (``cross_track_tolerance``). This is particularly useful for applications where precise alignment along a specific axis is more important than radial distance from the goal.

.. image:: /images/axis_goal_checker.png
   :alt: AxisGoalChecker Illustration
   :align: center

Parameters
**********

``<nav2_controller plugin>``: nav2_controller plugin name defined in the **goal_checker_plugin_id** parameter in :ref:`configuring_controller_server`.

:``<nav2_controller plugin>``.along_path_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======

    Description
        Tolerance for the projected distance along the path direction (m). This checks how far ahead or behind the goal the robot is when projected onto the path axis.

:``<nav2_controller plugin>``.cross_track_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======

    Description
        Tolerance for the perpendicular distance from the path direction (m). This checks how far to the left or right of the path axis the robot is.

:``<nav2_controller plugin>``.path_length_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 1.0
  ====== =======

    Description
        Maximum path length to consider for goal checking (m). If the remaining path length exceeds this value, the goal check is skipped. This prevents premature goal acceptance when far from the goal.

:``<nav2_controller plugin>``.is_overshoot_valid:

  ==== =======
  Type Default
  ---- -------
  bool false
  ==== =======

    Description
        Whether to allow overshooting past the goal along the path direction. When false (default), uses ``fabs(projected_distance) < along_path_tolerance`` for symmetric tolerance. When true, uses ``projected_distance < along_path_tolerance``, allowing the robot to be any distance past the goal but still requiring it to be within tolerance if before the goal.

Example Configuration
*********************

.. code-block:: yaml

    controller_server:
      ros__parameters:
        goal_checker_plugins: ["goal_checker"]
        goal_checker:
          plugin: "nav2_controller::AxisGoalChecker"
          along_path_tolerance: 0.20
          cross_track_tolerance: 0.15
          path_length_tolerance: 1.0
          is_overshoot_valid: false

How It Works
************

The AxisGoalChecker algorithm:

1. Extracts the last two poses from the path: ``before_goal_pose`` (second-to-last) and ``goal_pose`` (last)
2. Calculates the path direction (``end_of_path_yaw``) from these two points
3. Computes the angle from the robot to the goal (``robot_to_goal_yaw``)
4. Projects the robot-to-goal distance onto the path direction to get:
   
   - ``projected_distance_to_goal``: distance along the path axis (using ``cos(projection_angle)``)
   - ``ortho_projected_distance_to_goal``: distance perpendicular to the path axis (using ``sin(projection_angle)``)

5. Checks if both projections are within their respective tolerances

This approach allows the robot to reach the goal from various angles while maintaining alignment with the final path segment, making it ideal for applications requiring precise approach directions (e.g., docking, pallet pickup, or narrow corridor navigation).

Fallback Behavior
*****************

If the path contains only a single pose (no ``before_goal_pose`` available), the goal checker reverts to a simple Euclidean distance check where both ``along_path_tolerance`` and ``cross_track_tolerance`` must be satisfied.
