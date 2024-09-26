.. _configuring_vector_pursuit_controller:

Vector Pursuit Controller
#########################

Source code on Github_.

.. _Github: https://github.com/blackcoffeerobotics/vector_pursuit_controller

The Vector Pursuit controller implements a path-tracking algorithm based on the theory of screws. Unlike most other controllers, Vector Pursuit incorporates the path's orientation with the position when calculating velocity commands.
The use of screws unifies rotation and translation, making the controller more robust and geometrically meaningful, especially in complex, dynamic environments. This enables the robot to follow the path more smoothly and accurately, especially when navigating tight corners or moving at high speeds.
By adjusting linear velocities based on path curvature, the controller minimizes overshoot in sharp turns, enhancing safety and stability. 

See the package's ``README`` for more complete information.


Vector Pursuit Parameters
*********************************

:k:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         8.0 
  ============== ===========================

  Description
    k is a constant that relates the rotation and translation times when calculating the resultant screw(as a sum of the rotation and translation screws). The relationship is rotation_time = k * translation_time. Increasing k will put more weightage on faster translation and decreasing k will, in turn, apply more weightage on faster rotation.

:desired_linear_vel:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.5 
  ============== ===========================

  Description
    The desired maximum linear velocity (m/s).
:min_turning_radius:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         1.0 
  ============== ===========================

  Description
    The minimum achievable turning radius(m) of the robot. Remember, sharper turns have smaller radii.

:lookahead_dist:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.6
  ============== =============================

  Description
    The lookahead distance (m) to find the lookahead point.

:min_lookahead_dist:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.3 
  ============== =============================

  Description
    The minimum lookahead distance (m) threshold for adaptive lookahead point.

:max_lookahead_dist:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.9 
  ============== =============================

  Description
    The maximum lookahead distance (m) threshold for adaptive lookahead point.

:lookahead_time:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         1.5
  ============== =============================

  Description
    The time (s) to integrate the current linear velocity to get the scaled lookahead distance for adaptive lookahead point.

:rotate_to_heading_angular_vel:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         1.8            
  ============== =============================

  Description
    Angular velocity for rotating to heading.

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
    Enable velocity adaptive ``lookahead_distance``.

:min_approach_linear_velocity:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.05            
  ============== =============================

  Description
    The minimum velocity approach aware linear velocity scaling can produce. 

:approach_velocity_scaling_dist:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         1.0            
  ============== =============================

  Description
    The distance to goal at which velocity scaling will begin. Set to 0 to disable.

:use_collision_detection:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           true           
  ============== =============================

  Description
    Enable/disable collision detection.

:max_allowed_time_to_collision_up_to_target:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         1.0          
  ============== =============================

  Description
    Maximum time (s) allowed for collision checking.

:use_cost_regulated_linear_velocity_scaling:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           true            
  ============== =============================

  Description
    Whether to slowdown in close proximity to obstacles.

:cost_scaling_dist:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.6            
  ============== =============================

  Description
    Distance for cost-based velocity scaling.

:cost_scaling_gain:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         1.0            
  ============== =============================

  Description
    Multiplicative factor for cost-based velocity scaling.

:inflation_cost_scaling_factor:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         3.0            
  ============== =============================

  Description
    Factor for inflation cost scaling.

:min_linear_velocity:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.05            
  ============== =============================

  Description
    The minimum speed (m/s) robot must run at. Must be ``> 0.01``.

:use_rotate_to_heading:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           true            
  ============== =============================

  Description
    Enable/disable rotate-to-heading behavior. Will override reversing if both are enabled.

:allow_reversing:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           false            
  ============== =============================

  Description
    Enable/Disable reversing movement. Will move in reverse if the lookahead point is behind the robot.

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
    Maximum allowable angular acceleration (rad/s^2).

:max_linear_accel:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         2.0         
  ============== =============================

  Description
    Maximum allowable linear acceleration (m/s^2).

:max_lateral_accel:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.5          
  ============== =============================

  Description
    Maximum allowable acceleration (m/s^2) perpendicular to axis of desired translation motion. This limit is used to restrict the maximum speed for a given path curvature.

:use_interpolation:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           true         
  ============== =============================

  Description
    Calculate lookahead point exactly at the lookahead distance. Otherwise select a discrete point on the path.

:use_heading_from_path:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           false         
  ============== =============================

  Description
    If set to true, uses the orientation from the path poses otherwise compute appropriate orientations. Only set to true if using a planner that takes robot heading into account like Smac Planner.

:max_robot_pose_search_dist:

  ============== =================================================
  Type           Default
  -------------- -------------------------------------------------
  double         Local costmap max extent (max(width, height) / 2)
  ============== =================================================

  Description
    Maximum search distance for target poses along the global plan.

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
      failure_tolerance: 0.3
      progress_checker_plugin: "progress_checker"
      goal_checker_plugins: ["general_goal_checker"]
      controller_plugins: ["FollowPath"]

      # Progress checker parameters
      progress_checker:
        plugin: "nav2_controller::SimpleProgressChecker"
        required_movement_radius: 0.25
        movement_time_allowance: 10.0

      # Goal checker parameters
      general_goal_checker:
        plugin: "nav2_controller::SimpleGoalChecker"
        xy_goal_tolerance: 0.25
        yaw_goal_tolerance: 0.25
        stateful: True
      
      FollowPath:
        plugin: "vector_pursuit_controller::VectorPursuitController"
        k: 5.0
        desired_linear_vel: 0.5
        min_turning_radius: 0.25
        lookahead_dist: 1.0
        min_lookahead_dist: 0.5
        max_lookahead_dist: 1.5
        lookahead_time: 1.5
        rotate_to_heading_angular_vel: 0.5
        transform_tolerance: 0.1
        use_velocity_scaled_lookahead_dist: false
        min_linear_velocity: 0.0
        min_approach_linear_velocity: 0.05
        approach_velocity_scaling_dist: 0.5
        max_allowed_time_to_collision_up_to_target: 1.0
        use_collision_detection: true
        use_cost_regulated_linear_velocity_scaling: true
        cost_scaling_dist: 0.5
        cost_scaling_gain: 1.0
        inflation_cost_scaling_factor: 3.0
        use_rotate_to_heading: true
        allow_reversing: false
        rotate_to_heading_min_angle: 0.5
        max_angular_accel: 3.0
        max_linear_accel: 2.0
        max_lateral_accel: 0.2
        max_robot_pose_search_dist: 10.0
        use_interpolation: true
        use_heading_from_path: false
        approach_velocity_scaling_dist: 1.0

Acknowledgements
****************

We acknowledge the contributions of:

1. The author of `Vector Pursuit Path Tracking for Autonomous Ground Vehicles <https://apps.dtic.mil/sti/pdfs/ADA468928.pdf>`_, Jeffrey S. Wіt.
2. The `Nav2 Regulated Pure Pursuit Controller project <https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller>`_.
