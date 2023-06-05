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

If you use the Regulated Pure Pursuit Controller algorithm or software from this repository, please cite this work in your papers:

- S. Macenski, S. Singh, F. Martin, J. Gines, `Regulated Pure Pursuit for Robot Path Tracking <https://arxiv.org/abs/2305.20026>`_. Autonomous Robots, 2023.

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

    Note: both ``use_rotate_to_heading`` and ``allow_reversing`` cannot be set to ``true`` at the same time as it would result in ambiguous situations.

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
    Maximum allowable angular acceleration (rad/s/s) while rotating to heading, if ``use_rotate_to_heading`` is ``true``.

:max_robot_pose_search_dist:

  ============== =================================================
  Type           Default
  -------------- -------------------------------------------------
  double         Local costmap max extent (max(width, height) / 2)
  ============== =================================================

  Description
    Upper bound on integrated distance along the global plan to search for the closest pose to the robot pose. This should be left as the default unless there are paths with loops and intersections that do not leave the local costmap, in which case making this value smaller is necessary to prevent shortcutting. If set to ``-1``, it will use the maximum distance possible to search every point on the path for the nearest path point.

:use_interpolation:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  bool           true                         
  ============== =============================

  Description
    Enable linear interpolation between poses for lookahead point selection. Leads to smoother commanded linear and angular velocities.

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
        desired_linear_vel: 0.5
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
        curvature_lookahead_dist: 0.25
        use_cost_regulated_linear_velocity_scaling: false
        regulated_linear_scaling_min_radius: 0.9
        regulated_linear_scaling_min_speed: 0.25
        use_rotate_to_heading: true
        allow_reversing: false
        rotate_to_heading_min_angle: 0.785
        max_angular_accel: 3.2
        max_robot_pose_search_dist: 10.0
        use_interpolation: false
