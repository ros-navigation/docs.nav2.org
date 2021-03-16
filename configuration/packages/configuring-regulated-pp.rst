.. _configuring_regulated_pure_puruit:

Regulated Pure Pursuit
######################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_regulated_pure_pursuit_controller

The Regulated Pure Pursuit controller implements a variation on the Pure Pursuit controller that specifically targeting service / industrial robot needs.
It regulates the linear velocities by curvature of the path to help reduce overshoot at high speeds around blind corners allowing operations to be much more safe.
It also better follows paths than any other variation currently available of Pure Pursuit.
It also has heuristics to slow in proximity to other obstacles so that you can slow the robot automatically when nearby potential collisions.
It also implements the Adaptive lookahead point features to be scaled by velocities to enable more stable behavior in a larger range of translational speeds.

See the package's ``README`` for more complete information.


Regulated Pure Pursuit Parameters
*********************************

:desired_linear_vel:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.5 
  ============== ===========================

  Description
    The desired maximum linear velocity (m/s) to use.

:max_linear_accel:

  ============== ===================================
  Type           Default                                               
  -------------- -----------------------------------
  double         2.5            
  ============== ===================================

  Description
    Maximum acceleration for linear velocity (m/s/s)

:max_linear_decel:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         2.5
  ============== =============================

  Description
    Maximum deceleration for linear velocity (m/s/s)

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
  double         0.4            
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
    The minimum velocity (m/s) threshold to apply when approaching the goal to ensure progress, when ``use_approach_linear_velocity_scaling`` is ``true``.

:use_approach_linear_velocity_scaling:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           true            
  ============== =============================

  Description
    Whether to scale the linear velocity down on approach to the goal for a smooth stop.

:max_allowed_time_to_collision:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         1.0          
  ============== =============================

  Description
    The time (s) to project a velocity command forward to check for collisions.

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

:regulated_linear_scaling_min_radius:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.90       
  ============== =============================

  Description
    The turning radius (m) for which the regulation features are triggered when ``use_regulated_linear_velocity_scaling`` is ``tru``. Remember, sharper turns have smaller radii.

:regulated_linear_scaling_min_speed:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.25            
  ============== =============================

  Description
    The minimum speed (m/s) for which any of the regulated heuristics can send, to ensure process is still achievable even in high cost spaces with high curvature.

:use_rotate_to_heading:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           true            
  ============== =============================

  Description
    Whether to enable rotating to rough heading and goal orientation when using holonomic planners. Recommended on for all robot types that can rotate in place.

:rotate_to_heading_min_angle:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.785            
  ============== =============================

  Description
    The difference in the path orientation and the starting robot orientation (radians) to trigger a rotate in place, if ``use_rotate_to_heading`` is ``true``.

:max_angular_accel:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         3.2          
  ============== =============================

  Description
    Maximum allowable angular acceleration (m/s/s) while rotating to heading, if ``use_rotate_to_heading`` is ``true``.

Example
*******
.. code-block:: yaml

  controller_server:
    ros__parameters:
      use_sim_time: True
      controller_frequency: 20.0
      min_x_velocity_threshold: 0.001
      min_y_velocity_threshold: 0.5
      min_theta_velocity_threshold: 0.001
      progress_checker_plugin: "progress_checker"
      goal_checker_plugin: "goal_checker"
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
        desired_linear_vel: 0.5
        max_linear_accel: 2.5
        max_linear_decel: 2.5
        lookahead_dist: 0.6
        min_lookahead_dist: 0.3
        max_lookahead_dist: 0.9
        lookahead_time: 1.5
        rotate_to_heading_angular_vel: 1.8
        transform_tolerance: 0.1
        use_velocity_scaled_lookahead_dist: false
        min_approach_linear_velocity: 0.05
        use_approach_linear_velocity_scaling: true
        max_allowed_time_to_collision: 1.0
        use_regulated_linear_velocity_scaling: true
        use_cost_regulated_linear_velocity_scaling: false
        regulated_linear_scaling_min_radius: 0.9
        regulated_linear_scaling_min_speed: 0.25
        use_rotate_to_heading: true
        rotate_to_heading_min_angle: 0.785
        max_angular_accel: 3.2
