.. _configuring_rotation_shim:

Rotation Shim Controller
########################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_rotation_shim_controller

The ``nav2_rotation_shim_controller`` will check the rough heading difference with respect to the robot and a newly received path. If within a threshold, it will pass the request onto the ``primary_controller`` to execute the task. If it is outside of the threshold, this controller will rotate the robot in place towards that path heading. Once it is within the tolerance, it will then pass off control-execution from this rotation shim controller onto the primary controller plugin. At this point, the robot's main plugin will take control for a smooth hand off into the task. 

The ``RotationShimController`` is most suitable for:

- Robots that can rotate in place, such as differential and omnidirectional robots.
- Preference to rotate in place when starting to track a new path that is at a significantly different heading than the robot's current heading -- or when tuning your controller for its task makes tight rotations difficult.
- Using planners that are non-kinematically feasible, such as NavFn, Theta\*, or Smac 2D (Feasible planners such as Smac Hybrid-A* and State Lattice will start search from the robot's actual starting heading, requiring no rotation since their paths are guaranteed drivable by physical constraints). 

See the package's ``README`` for more complete information.

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="708" height="400" src="https://www.youtube.com/embed/t-g2CBGByEw?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
      </div>
    </h1>

Rotation Shim Controller Parameters
***********************************

:angular_dist_threshold:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.785
  ============== ===========================

  Description
    Maximum angular distance, in radians, away from the path heading to trigger rotation until within.

:forward_sampling_distance:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.5
  ============== =============================

  Description
    Forward distance, in meters, along path to select a sampling point to use to approximate path heading

:rotate_to_heading_angular_vel:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         1.8 
  ============== =============================

  Description
    Angular rotational velocity, in rad/s, to rotate to the path heading

:primary_controller:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         N/A 
  ============== =============================

  Description
    Internal controller plugin to use for actual control behavior after rotating to heading

:max_angular_accel:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         3.2
  ============== =============================

  Description
    Maximum angular acceleration for rotation to heading (rad/s/s)

:simulate_ahead_time:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         1.0
  ============== =============================

  Description
    Time in seconds to forward simulate a rotation command to check for collisions. If a collision is found, forwards control back to the primary controller plugin.

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
        plugin: "nav2_rotation_shim_controller::RotationShimController"
        primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        angular_dist_threshold: 0.785
        forward_sampling_distance: 0.5
        rotate_to_heading_angular_vel: 1.8
        max_angular_accel: 3.2
        simulate_ahead_time: 1.0

        # Primary controller params can be placed here below
        # ...
