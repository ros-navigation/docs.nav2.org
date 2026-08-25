# DWB Controller { #dwb-controller-index }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_dwb_controller).

The DWB controller is the default controller. It is a fork of [David Lu's
controller](https://github.com/locusrobotics/robot_navigation/tree/master/dwb_local_planner)
modified for ROS 2 using the Dynamic Window Approach.

<span class="section-title">Controller</span>

<div class="grid cards bottom-align" markdown>

- :material-steering: **DWB Controller**

    ---
    Core DWB local planner parameters.

    [:octicons-arrow-right-24: Go][dwb-controller]

- :material-axis-arrow: **XYTheta Iterator**

    ---
    Velocity space sampling configuration.

    [:octicons-arrow-right-24: Go][xytheta-iterator]

- :material-cog: **Kinematic Parameters**

    ---
    Robot velocity and acceleration limits.

    [:octicons-arrow-right-24: Go][kinematic-parameters]

- :material-publish: **Publisher**

    ---
    Debug visualization publishers.

    [:octicons-arrow-right-24: Go][publisher]

</div>

<span class="section-title">Plugins</span>

The plugins listed below are inside the `dwb_plugins` namespace.

<div class="grid cards bottom-align" markdown>

- :material-speedometer: **LimitedAccelGenerator**

    ---
    Generates trajectories within acceleration limits.

    [:octicons-arrow-right-24: Go][limited-accel-generator]

- :material-chart-timeline: **StandardTrajectoryGenerator**

    ---
    Default trajectory sampling generator.

    [:octicons-arrow-right-24: Go][standard-trajectory-generator]

</div>

<span class="section-title">Trajectory Critics</span>

The trajectory critics listed below are inside the `dwb_critics` namespace.

<div class="grid cards bottom-align" markdown>

- :material-alert-circle: **BaseObstacleCritic**

    ---
    Penalizes trajectories near obstacles.

    [:octicons-arrow-right-24: Go][base-obstacle-critic]

- :material-target: **GoalAlignCritic**

    ---
    Scores alignment with the goal heading.

    [:octicons-arrow-right-24: Go][goal-align-critic]

- :material-map-marker-distance: **GoalDistCritic**

    ---
    Scores proximity to the goal position.

    [:octicons-arrow-right-24: Go][goal-dist-critic]

- :material-shoe-print: **ObstacleFootprintCritic**

    ---
    Checks full robot footprint against obstacles.

    [:octicons-arrow-right-24: Go][obstacle-footprint-critic]

- :material-sine-wave: **OscillationCritic**

    ---
    Penalizes oscillating back-and-forth motion.

    [:octicons-arrow-right-24: Go][oscillation-critic]

- :material-road: **PathAlignCritic**

    ---
    Scores alignment with the global path heading.

    [:octicons-arrow-right-24: Go][path-align-critic]

- :material-ruler: **PathDistCritic**

    ---
    Scores proximity to the global path.

    [:octicons-arrow-right-24: Go][path-dist-critic]

- :material-arrow-up: **PreferForwardCritic**

    ---
    Penalizes backward and lateral motion.

    [:octicons-arrow-right-24: Go][prefer-forward-critic]

- :material-rotate-right: **RotateToGoalCritic**

    ---
    Encourages in-place rotation to match goal heading.

    [:octicons-arrow-right-24: Go][rotate-to-goal-critic]

- :material-rotate-3d-variant: **TwirlingCritic**

    ---
    Penalizes unnecessary spinning during transit.

    [:octicons-arrow-right-24: Go][twirling-critic]

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
