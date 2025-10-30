.. _configuring_regulated_pure_puruit:

Regulated Pure Pursuit
######################

Source code on Github_.

.. _Github: https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller

The Regulated Pure Pursuit controller implements a variation on the Pure Pursuit controller that specifically targeting service / industrial robot needs.
It regulates the linear velocities by curvature of the path to help reduce overshoot at high speeds around blind corners allowing operations to be much more safe.
It also better follows paths than any other variation currently available of Pure Pursuit.
It also has heuristics to slow in proximity to other obstacles so that you can slow the robot automatically when nearby potential collisions.
It also implements the Adaptive lookahead point features to be scaled by velocities to enable more stable behavior in a larger range of translational speeds.
It also considers the robot’s velocity and acceleration constraints during velocity command computation using `Dynamic Window Pure Pursuit <https://github.com/Decwest/nav2_dynamic_window_pure_pursuit_controller/blob/main/algorithm.md>`_ algorithm.

See the package's ``README`` for more complete information.

If you use the Regulated Pure Pursuit Controller algorithm or software from this repository, please cite this work in your papers:

- S. Macenski, S. Singh, F. Martin, J. Gines, `Regulated Pure Pursuit for Robot Path Tracking <https://arxiv.org/abs/2305.20026>`_. Autonomous Robots, 2023.

Regulated Pure Pursuit Parameters
*********************************

:max_linear_vel:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         0.5
  ============== ===========================

  Description
    The maximum linear velocity (m/s) to use.

:min_linear_vel:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         0.0
  ============== ===========================

  Description
    The minimum linear velocity (m/s) to use.

:max_angular_vel:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         1.0
  ============== ===========================

  Description
    The maximum angular velocity (rad/s) to use.

:min_angular_vel:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         -1.0
  ============== ===========================

  Description
    The minimum angular velocity (rad/s) to use.

:max_linear_accel:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         0.5
  ============== ===========================

  Description
    The maximum linear acceleration (m/s^2) to use.

:max_linear_decel:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         0.5
  ============== ===========================

  Description
    The maximum linear deceleration (m/s^2) to use.

:max_angular_accel:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         1.0
  ============== ===========================

  Description
    The maximum angular acceleration (rad/s^2) to use.

:max_angular_decel:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         1.0
  ============== ===========================

  Description
    The maximum angular deceleration (rad/s^2) to use.

:lookahead_dist:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.6
  ============== =============================

  Description
    The lookahead distance (m) to use to find the lookahead point when ``use_velocity_scaled_lookahead_dist`` is ``false``.

:min_lookahead_dist:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.3
  ============== =============================

  Description
    The minimum lookahead distance (m) threshold when ``use_velocity_scaled_lookahead_dist`` is ``true``.

:max_lookahead_dist:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.9
  ============== =============================

  Description
    The maximum lookahead distance (m) threshold when ``use_velocity_scaled_lookahead_dist`` is ``true``.

:lookahead_time:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         1.5
  ============== =============================

  Description
    The time (s) to project the velocity by when ``use_velocity_scaled_lookahead_dist`` is ``true``. Also known as the lookahead gain.

:rotate_to_heading_angular_vel:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         1.8
  ============== =============================

  Description
    If ``use_rotate_to_heading`` is ``true``, this is the angular velocity to use.

:transform_tolerance:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.1
  ============== =============================

  Description
    The TF transform tolerance (s).

:use_velocity_scaled_lookahead_dist:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           false
  ============== =============================

  Description
    Whether to use the velocity scaled lookahead distances or constant ``lookahead_distance``.

:min_approach_linear_velocity:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.05
  ============== =============================

  Description
    The minimum velocity (m/s) threshold to apply when approaching the goal to ensure progress. Must be ``> 0.01``.

:approach_velocity_scaling_dist:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.6
  ============== =============================

  Description
    The distance (m) left on the path at which to start slowing down. Should be less than the half the costmap width.

:use_collision_detection:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           true
  ============== =============================

  Description
    Whether to enable collision detection.

:max_allowed_time_to_collision_up_to_carrot:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         1.0
  ============== =============================

  Description
    The time (s) to project a velocity command forward to check for collisions when ``use_collision_detection`` is ``true``. Pre-``Humble``, this was ``max_allowed_time_to_collision``.

:use_regulated_linear_velocity_scaling:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           true
  ============== =============================

  Description
    Whether to use the regulated features for path curvature (e.g. slow on high curvature paths).

:use_cost_regulated_linear_velocity_scaling:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           true
  ============== =============================

  Description
    Whether to use the regulated features for proximity to obstacles (e.g. slow in close proximity to obstacles).

:cost_scaling_dist:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.6
  ============== =============================

  Description
    The minimum distance from an obstacle to trigger the scaling of linear velocity, if ``use_cost_regulated_linear_velocity_scaling`` is enabled. The value set should be smaller or equal to the ``inflation_radius`` set in the inflation layer of costmap, since inflation is used to compute the distance from obstacles.

:cost_scaling_gain:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         1.0
  ============== =============================

  Description
    A multiplier gain, which should be <= 1.0, used to further scale the speed when an obstacle is within ``cost_scaling_dist``. Lower value reduces speed more quickly.

:inflation_cost_scaling_factor:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         3.0
  ============== =============================

  Description
    The value of `cost_scaling_factor` set for the inflation layer in the local costmap. The value should be exactly the same for accurately computing distance from obstacles using the inflated cell values

:regulated_linear_scaling_min_radius:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.90
  ============== =============================

  Description
    The turning radius (m) for which the regulation features are triggered when ``use_regulated_linear_velocity_scaling`` is ``true``. Remember, sharper turns have smaller radii.

:regulated_linear_scaling_min_speed:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.25
  ============== =============================

  Description
    The minimum speed (m/s) for which any of the regulated heuristics can send, to ensure process is still achievable even in high cost spaces with high curvature. Must be ``> 0.1``.

:use_fixed_curvature_lookahead:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           false
  ============== =============================

  Description
    Whether to use a fixed lookahead distance to compute curvature from. Since a lookahead distance may be set to vary on velocity, it can introduce a reference cycle that can be problematic for large lookahead distances.

:curvature_lookahead_dist:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.6
  ============== =============================

  Description
    Distance to look ahead on the path to detect curvature.

:use_rotate_to_heading:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           true
  ============== =============================

  Description
    Whether to enable rotating to rough heading and goal orientation when using holonomic planners. Recommended on for all robot types that can rotate in place.

    Note: both ``use_rotate_to_heading`` and ``allow_reversing`` cannot be set to ``true`` at the same time as it would result in ambiguous situations.

:allow_reversing:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           false
  ============== =============================

  Description
    Enables the robot to drive in the reverse direction, when the path planned involves reversing (which is represented by orientation cusps). Variants of the smac_planner comes with the support of reversing. Checkout the :ref:`configuring_smac_planner` to know more.

:rotate_to_heading_min_angle:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.785
  ============== =============================

  Description
    The difference in the path orientation and the starting robot orientation (radians) to trigger a rotate in place, if ``use_rotate_to_heading`` is ``true``.

:use_cancel_deceleration:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           false
  ============== =============================

  Description
    Whether to use deceleration when the goal is canceled.

:cancel_deceleration:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         3.2
  ============== =============================

  Description
    Linear deceleration (m/s/s) to apply when the goal is canceled.

:max_robot_pose_search_dist:

  ============== =================================================
  Type           Default
  -------------- -------------------------------------------------
  double         Local costmap max extent (max(width, height) / 2)
  ============== =================================================

  Description
    Upper bound on integrated distance along the global plan to search for the closest pose to the robot pose. This should be left as the default unless there are paths with loops and intersections that do not leave the local costmap, in which case making this value smaller is necessary to prevent shortcutting. If set to ``-1``, it will use the maximum distance possible to search every point on the path for the nearest path point.

:interpolate_curvature_after_goal:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           false
  ============== =============================

  Description
    Interpolate a carrot after the goal dedicated to the curvate calculation (to avoid oscilaltions at the end of the path). For visualization, it will be published on the ``/curvature_lookahead_point`` topic similarly to ``/lookahead_point``

    Note: Needs ``use_fixed_curvature_lookahead`` to be ``true``

:stateful:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           true
  ============== =============================

  Description
    Enables stateful goal handling behavior. When set to true, the controller will persist the goal state once the robot reaches the XY tolerance. It will then focus on aligning to the goal heading without reverting to XY position corrections.

:allow_parameter_qos_overrides:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           true
  ============== =============================

  Description
    Whether to allow QoS profiles to be overwritten with parameterized values.

:min_distance_to_obstacle:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         -1.0
  ============== =============================

  Description
    The shortest distance at which the robot is allowed to be from an obstacle along its trajectory. Set <= 0.0 to disable. It is limited to maximum distance of lookahead distance selected.

:use_dynamic_window:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           true
  ============== =============================

  Description
    Whether to use the Dynamic Window Pure Pursuit (DWPP) Algorithm. This algorithm computes optimal path tracking velocity commands under velocity and acceleration constraints.

:velocity_feedback:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  std::string    "OPEN_LOOP"
  ============== =============================

  Description
    How the current velocity is obtained during dynamic window computation. "OPEN_LOOP" uses the last commanded velocity (recommended). "CLOSED_LOOP" uses odometry velocity (may hinder proper acceleration/deceleration).



Example
*******
.. code-block:: yaml

  controller_server:
    ros__parameters:
      controller_frequency: 20.0
      min_x_velocity_threshold: 0.001
      min_y_velocity_threshold: 0.5
      min_theta_velocity_threshold: 0.001
      progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
      goal_checker_plugins: ["goal_checker"]
      controller_plugins: ["FollowPath"]

      progress_checker:
        plugin: "nav2_controller::SimpleProgressChecker"
        required_movement_radius: 0.5
        movement_time_allowance: 10.0
      goal_checker:
        plugin: "nav2_controller::SimpleGoalChecker"
        xy_goal_tolerance: 0.25
        yaw_goal_tolerance: 0.25
        stateful: True
      FollowPath:
        plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        max_linear_vel: 0.5
        min_linear_vel: 0.0
        max_angular_vel: 1.0
        min_angular_vel: -1.0
        max_linear_accel: 0.5
        max_linear_decel: 0.5
        max_angular_accel: 1.0
        max_angular_decel: 1.0
        lookahead_dist: 0.6
        min_lookahead_dist: 0.3
        max_lookahead_dist: 0.9
        lookahead_time: 1.5
        rotate_to_heading_angular_vel: 1.8
        transform_tolerance: 0.1
        use_velocity_scaled_lookahead_dist: false
        min_approach_linear_velocity: 0.05
        approach_velocity_scaling_dist: 0.6
        use_collision_detection: true
        max_allowed_time_to_collision_up_to_carrot: 1.0
        use_regulated_linear_velocity_scaling: true
        use_fixed_curvature_lookahead: false
        curvature_lookahead_dist: 0.6
        use_cost_regulated_linear_velocity_scaling: false
        interpolate_curvature_after_goal: false
        cost_scaling_dist: 0.3
        cost_scaling_gain: 1.0
        inflation_cost_scaling_factor: 3.0
        regulated_linear_scaling_min_radius: 0.9
        regulated_linear_scaling_min_speed: 0.25
        use_rotate_to_heading: true
        allow_reversing: false
        rotate_to_heading_min_angle: 0.785
        max_robot_pose_search_dist: 10.0
        min_distance_to_obstacle: 0.0
        use_dynamic_window: true
        velocity_feedback: "OPEN_LOOP"
