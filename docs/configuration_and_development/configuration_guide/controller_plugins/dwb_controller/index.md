# DWB Controller { #dwb-controller-index }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/main/nav2_dwb_controller).

The DWB controller is the default controller. It is a fork of [David Lu's
controller](https://github.com/locusrobotics/robot_navigation/tree/master/dwb_local_planner)
modified for ROS 2 using the Dynamic Window Approach.

<span class="section-title">Controller</span>

<div class="grid" markdown>

[DWB Controller][dwb-controller]{ .md-button .md-button--primary }
[XYTheta Iterator][xytheta-iterator]{ .md-button .md-button--primary }
[Kinematic Parameters][kinematic-parameters]{ .md-button .md-button--primary }
[Publisher][publisher]{ .md-button .md-button--primary }

</div>

<span class="section-title">Plugins</span>

The plugins listed below are inside the `dwb_plugins` namespace.

<div class="grid" markdown>

[LimitedAccelGenerator][limited-accel-generator]{ .md-button .md-button--primary }
[StandardTrajectoryGenerator][standard-trajectory-generator]{ .md-button .md-button--primary }

</div>

<span class="section-title">Trajectory Critics</span>

The trajectory critics listed below are inside the `dwb_critics` namespace.

<div class="grid" markdown>

[BaseObstacleCritic][base-obstacle-critic]{ .md-button .md-button--primary }
[GoalAlignCritic][goal-align-critic]{ .md-button .md-button--primary }
[GoalDistCritic][goal-dist-critic]{ .md-button .md-button--primary }
[ObstacleFootprintCritic][obstacle-footprint-critic]{ .md-button .md-button--primary }
[OscillationCritic][oscillation-critic]{ .md-button .md-button--primary }
[PathAlignCritic][path-align-critic]{ .md-button .md-button--primary }
[PathDistCritic][path-dist-critic]{ .md-button .md-button--primary }
[PreferForwardCritic][prefer-forward-critic]{ .md-button .md-button--primary }
[RotateToGoalCritic][rotate-to-goal-critic]{ .md-button .md-button--primary }
[TwirlingCritic][twirling-critic]{ .md-button .md-button--primary }

</div>

<span class="section-title">Example</span>

```yaml
controller_server:
  ros__parameters:
    # controller server parameters (see Controller Server for more info)
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
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
      xy_goal_tolerance: 0.25
      path_length_tolerance: 1.0
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      limit_vel_cmd_in_traj: False
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
```
