.. _configuring_dwb_controller:

DWB Controller
##############

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_dwb_controller

The DWB controller is the default controller. It is a fork of `David Lu's
controller <https://github.com/locusrobotics/robot_navigation/tree/master/dwb_local_planner>`_
modified for ROS 2.

Controller
**********
.. toctree::
  :maxdepth: 1

  dwb-params/controller.rst
  dwb-params/iterator.rst
  dwb-params/kinematic.rst
  dwb-params/visualization.rst

Plugins
*******

The plugins listed below are inside the ``dwb_plugins`` namespace.

.. toctree::
  :maxdepth: 1

  dwb-plugins/limited_accel_generator.rst
  dwb-plugins/standard_traj_generator.rst


Trajectory Critics
******************

The trajectory critics listed below are inside the ``dwb_critics`` namespace.

.. toctree::
  :maxdepth: 1

  trajectory_critics/base_obstacle.rst
  trajectory_critics/goal_align.rst
  trajectory_critics/goal_dist.rst
  trajectory_critics/obstacle_footprint.rst
  trajectory_critics/oscillation.rst
  trajectory_critics/path_align.rst
  trajectory_critics/path_dist.rst
  trajectory_critics/prefer_forward.rst
  trajectory_critics/rotate_to_goal.rst
  trajectory_critics/twirling.rst

Example
*******
.. code-block:: yaml

    controller_server:
      ros__parameters:
        # controller server parameters (see Controller Server for more info)
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
        # DWB controller parameters
        FollowPath:
          plugin: "dwb_core::DWBLocalPlanner"
          debug_trajectory_details: True
          min_vel_x: 0.0
          min_vel_y: 0.0
          max_vel_x: 0.26
          max_vel_y: 0.0
          max_vel_theta: 1.0
          min_speed_xy: 0.0
          max_speed_xy: 0.26
          min_speed_theta: 0.0
          acc_lim_x: 2.5
          acc_lim_y: 0.0
          acc_lim_theta: 3.2
          decel_lim_x: -2.5
          decel_lim_y: 0.0
          decel_lim_theta: -3.2
          vx_samples: 20
          vy_samples: 5
          vtheta_samples: 20
          sim_time: 1.7
          linear_granularity: 0.05
          angular_granularity: 0.025
          transform_tolerance: 0.2
          xy_goal_tolerance: 0.25
          trans_stopped_velocity: 0.25
          short_circuit_trajectory_evaluation: True
          stateful: True
          critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
          BaseObstacle.scale: 0.02
          PathAlign.scale: 32.0
          GoalAlign.scale: 24.0
          PathAlign.forward_point_distance: 0.1
          GoalAlign.forward_point_distance: 0.1
          PathDist.scale: 32.0
          GoalDist.scale: 24.0
          RotateToGoal.scale: 32.0
          RotateToGoal.slowing_factor: 5.0
          RotateToGoal.lookahead_time: -1.0

