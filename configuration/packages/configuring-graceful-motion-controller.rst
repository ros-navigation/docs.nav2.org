.. _configuring_graceful_motion_controller:

Graceful Controller
###################

Source code on Github_.

.. _Github: https://github.com/ros-navigation/navigation2/tree/main/nav2_graceful_controller

The graceful controller implements a controller based on the works of Jong Jin Park and Benjamin Kuipers in "A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment" (ICRA 2011). In this implementation, a `motion_target` is set at a distance away from the robot that is exponentially stable to generate a smooth trajectory for the robot to follow.

See the package's ``README`` for more complete information.

Graceful Controller Parameters
******************************

:transform_tolerance:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.1 
  ============== ===========================

  Description
    The TF transform tolerance (s).

:motion_target_dist:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         0.6
  ============== =============================

  Description
    The lookahead distance (m) to use to find the motion_target point.

:max_robot_pose_search_dist:

  ============== =================================================
  Type           Default
  -------------- -------------------------------------------------
  double         Local costmap max extent (max(width, height) / 2)
  ============== =================================================

  Description
    Upper bound on integrated distance along the global plan to search for the closest pose to the robot pose. This should be left as the default unless there are paths with loops and intersections that do not leave the local costmap, in which case making this value smaller is necessary to prevent shortcutting. If set to ``-1``, it will use the maximum distance possible to search every point on the path for the nearest path point.

:k_phi:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         3.0 
  ============== =============================

  Description
    Ratio of the rate of change in phi to the rate of change in r. Controls the convergence of the slow subsystem. If this value is equal to zero, the controller will behave as a pure waypoint follower. A high value offers extreme scenario of pose-following where theta is reduced much faster than r.

:k_delta:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         2.0 
  ============== =============================

  Description
    Constant factor applied to the heading error feedback. Controls the convergence of the fast subsystem. The bigger the value, the robot converge faster to the reference heading.

:beta:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         0.2 
  ============== =============================

  Description
    Constant factor applied to the path curvature. This value must be positive. Determines how fast the velocity drops when the curvature increases.

:lambda:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         2.0 
  ============== =============================

  Description
   Constant factor applied to the path curvature. This value must be greater or equal to 1. Determines the sharpness of the curve: higher lambda implies sharper curves.

:v_linear_min:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         0.1      
  ============== =============================

  Description
    Minimum linear velocity (m/s).

:v_linear_max:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         0.5 
  ============== =============================

  Description
    Maximum linear velocity (m/s).

:v_angular_max:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         1.0 
  ============== =============================

  Description
    Maximum angular velocity (rad/s) produced by the control law.

:slowdown_radius:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         1.5 
  ============== =============================

  Description
    Radius (m) around the goal pose in which the robot will start to slow down.

:initial_rotation:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  bool           true 
  ============== =============================

  Description
    Enable a rotation in place to the goal before starting the path. The control law may generate large sweeping arcs to the goal pose, depending on the initial robot orientation and ``k_phi``, ``k_delta``.

:initial_rotation_min_angle:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         0.75 
  ============== =============================

  Description
    The difference in the path orientation and the starting robot orientation to trigger a rotate in place, if ``initial_rotation`` is enabled.

:final_rotation:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  bool           true 
  ============== =============================

  Description
    Similar to ``initial_rotation``, the control law can generate large arcs when the goal orientation is not aligned with the path. If this is enabled, the final pose will be ignored and the robot will follow the orientation of he path and will make a final rotation in place to the goal orientation.

:rotation_scaling_factor:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  double         0.5 
  ============== =============================

  Description
    The scaling factor applied to the rotation in place velocity.

:allow_backward:

  ============== =============================
  Type           Default                      
  -------------- -----------------------------
  bool           false 
  ============== =============================

  Description
    Whether to allow the robot to move backward.

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
        plugin: nav2_graceful_controller::GracefulController
        transform_tolerance: 0.1
        motion_target_dist: 0.6
        initial_rotation: true
        initial_rotation_min_angle: 0.75
        final_rotation: true
        allow_backward: false
        k_phi: 3.0
        k_delta: 2.0
        beta: 0.4
        lambda: 2.0
        v_linear_min: 0.1
        v_linear_max: 1.0
        v_angular_max: 5.0
        slowdown_radius: 1.5
