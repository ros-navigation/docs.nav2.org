.. _configuring_nav2_controller_feasible_path_handler_plugin:

FeasiblePathHandler
===================

Transforms the global plan into the local costmap frame, prunes it to the relevant portion
within the costmap bounds, and handles in-place rotation and cusp pruning.

Parameters
**********

``<nav2_controller plugin>``: nav2_controller plugin name defined in the **path_handler_plugin_id** parameter in :ref:`configuring_controller_server`.

:``<nav2_controller plugin>``.reject_unit_path:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  bool           false
  ============== ===========================

  Description
    If enabled, the path handler will reject a path that contains only a single pose.

:``<nav2_controller plugin>``.prune_distance:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         2.0
  ============== ===========================

  Description
    Distance ahead of nearest point on path to robot to prune path to (m). This distance should be at least as great as the furthest distance of interest by a critic (i.e. for maximum velocity projection forward, threshold to consider).

:``<nav2_controller plugin>``.max_robot_pose_search_dist:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         Costmap size / 2
  ============== ===========================

  Description
    Max integrated distance ahead of robot pose to search for nearest path point in case of path looping.

:``<nav2_controller plugin>``.enforce_path_inversion:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  bool           false
  ============== ===========================

  Description
    If true, it will prune paths containing cusping points for segments changing directions (e.g. path inversions) such that the controller will be forced to change directions at or very near the planner's requested inversion point. In addition, these cusping points will also be treated by the critics as local goals that the robot will attempt to reach. This is targeting Smac Planner users with feasible paths who need their robots to switch directions where specifically requested.

:``<nav2_controller plugin>``.enforce_path_rotation:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  bool           false
  ============== ===========================

  Description
    If true, the controller will detect in-place rotation segments (where translation is near zero) and prune the remaining poses after the rotation point. This forces the robot to explicitly perform the rotation before proceeding along the rest of the path.
    This is particularly useful for feasible planners (e.g., Smac Planner) where direction changes are intentionally introduced and must be respected during execution.

:``<nav2_controller plugin>``.minimum_rotation_angle:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         0.785
  ============== ===========================

  Description
    The minimum accumulated rotation (in radians) required to classify a segment as an in-place rotation. 0.785 rad = 45 deg.

:``<nav2_controller plugin>``.inversion_xy_tolerance:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         0.2
  ============== ===========================

  Description
    Cartesian proximity (m) to path inversion point to be considered "achieved" to pass on the rest of the path after path inversion.

:``<nav2_controller plugin>``.inversion_yaw_tolerance:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         0.4
  ============== ===========================

  Description
    Angular proximity (radians) to path inversion point to be considered "achieved" to pass on the rest of the path after path inversion. 0.4 rad = 23 deg.

Example
*******
.. code-block:: yaml

  path_handler:
      plugin: "nav2_controller::FeasiblePathHandler"
      prune_distance: 2.0
      enforce_path_inversion: True
      enforce_path_rotation: False
      inversion_xy_tolerance: 0.2
      inversion_yaw_tolerance: 0.4
      minimum_rotation_angle: 0.785
      reject_unit_path: False
