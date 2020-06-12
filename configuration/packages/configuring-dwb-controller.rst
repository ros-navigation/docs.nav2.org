.. _configuring_dwb_controller:

DWB Controller
##############

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_dwb_controller

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
  dwb-plugins/simple_goal_checker.rst
  dwb-plugins/standard_traj_generator.rst
  dwb-plugins/stopped_goal_checker.rst


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
        controller_plugin_types: ["dwb_core::DWBLocalPlanner"]
        controller_plugin_ids: ["FollowPath"]
        min_x_velocity_threshold: 0.001
        min_y_velocity_threshold: 0.5
        min_theta_velocity_threshold: 0.001

        # DWB controller parameters
        FollowPath.debug_trajectory_details: True
        FollowPath.min_vel_x: 0.0
        FollowPath.min_vel_y: 0.0
        FollowPath.max_vel_x: 0.26
        FollowPath.max_vel_y: 0.0
        FollowPath.max_vel_theta: 1.0
        FollowPath.min_speed_xy: 0.0
        FollowPath.max_speed_xy: 0.26
        FollowPath.min_speed_theta: 0.0
        FollowPath.acc_lim_x: 2.5
        FollowPath.acc_lim_y: 0.0
        FollowPath.acc_lim_theta: 3.2
        FollowPath.decel_lim_x: -2.5
        FollowPath.decel_lim_y: 0.0
        FollowPath.decel_lim_theta: -3.2
        FollowPath.vx_samples: 20
        FollowPath.vy_samples: 5
        FollowPath.vtheta_samples: 20
        FollowPath.sim_time: 1.7
        FollowPath.linear_granularity: 0.05
        FollowPath.angular_granularity: 0.025
        FollowPath.transform_tolerance: 0.2
        FollowPath.xy_goal_tolerance: 0.25
        FollowPath.trans_stopped_velocity: 0.25
        FollowPath.short_circuit_trajectory_evaluation: True
        FollowPath.stateful: True
        FollowPath.critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
        FollowPath.BaseObstacle.scale: 0.02
        FollowPath.PathAlign.scale: 32.0
        FollowPath.GoalAlign.scale: 24.0
        FollowPath.PathAlign.forward_point_distance: 0.1
        FollowPath.GoalAlign.forward_point_distance: 0.1
        FollowPath.PathDist.scale: 32.0
        FollowPath.GoalDist.scale: 24.0
        FollowPath.RotateToGoal.scale: 32.0
        FollowPath.RotateToGoal.slowing_factor: 5.0
        FollowPath.RotateToGoal.lookahead_time: -1.0

