.. _configuring_dwb_controller:

DWB Controller
##########################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_dwb_controller

The DWB controller is the default controller. It is a fork of `David Lu's
controller <https://github.com/locusrobotics/robot_navigation/tree/master/dwb_local_planner>`_
modified for ROS 2.

Parameters
**********

:``<dwb plugin>``.critics:

  ============== =======
  Type           Default
  -------------- -------
  vector<string> N/A    
  ============== =======

  Description
    List of critic plugins to use.

:``<dwb plugin>``.default_critic_namespaces:

  ============== ===============
  Type           Default                                               
  -------------- ---------------
  vector<string> ["dwb_critics"]           
  ============== ===============

  Description
    Namespaces to load critics in.

:``<dwb plugin>``.prune_plan:

  ==== =======
  Type Default
  ---- -------
  bool true   
  ==== =======

  Description
    Whether to prune the path of old, passed points.

:``<dwb plugin>``.prune_distance:

  ====== =======
  Type   Default
  ------ -------
  double 1.0    
  ====== =======

  Description
    Distance (m) to prune backward until.

:``<dwb plugin>``.debug_trajectory_details:

  ==== =======
  Type Default                                     
  ---- -------
  bool false  
  ==== =======

  Description
    Publish debug information (on what topic???).

:``<dwb plugin>``.trajectory_generator_name:

  ====== ==========================================
  Type   Default                                               
  ------ ------------------------------------------
  string "dwb_plugins::StandardTrajectoryGenerator"            
  ====== ==========================================

  Description
    Trajectory generator plugin name.

:``<dwb plugin>``.goal_checker_name:

  ============== ================================
  Type           Default                                               
  -------------- --------------------------------
  string         "dwb_plugins::SimpleGoalChecker"           
  ============== ================================

  Description
    Goal checker plugin name.

:``<dwb plugin>``.transform_tolerance:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.1        
  ============== =============================

  Description
    TF transform tolerance (s).

:``<dwb plugin>``.short_circuit_trajectory_evaluation:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           true            
  ============== =============================

  Description
    	Stop evaluating scores after best score is found.

:``<dwb plugin>``.path_distance_bias (Legacy):

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         N/A            
  ============== =============================

  Description
    	Old version of PathAlign.scale, use that instead.

:``<dwb plugin>``.goal_distance_bias (Legacy):

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         N/A           
  ============== =============================

  Description
    Old version of GoalAlign.scale, use that instead.

:``<dwb plugin>``.occdist_scale (Legacy):

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         N/A            
  ============== =============================

  Description
    Old version of ObstacleFootprint.scale, use that instead.

:``<dwb plugin>``.max_scaling_factor (Legacy):

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         N/A         
  ============== =============================

  Description
    Old version of ObstacleFootprint.max_scaling_factor, use that instead.

:``<dwb plugin>``.scaling_speed (Legacy):

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         N/A           
  ============== =============================

  Description
    Old version of ObstacleFootprint.scaling_speed, use that instead.

:``<dwb plugin>``.PathAlign.scale:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         32.0    
  ============== =============================

  Description
    Scale for path align critic, overriding local default.

:``<dwb plugin>``.GoalAlign.scale:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         24.0          
  ============== =============================

  Description
    Scale for goal align critic, overriding local default.

:``<dwb plugin>``.PathDist.scale:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         32.0           
  ============== =============================

  Description
    Scale for path distance critic, overriding local default.

:``<dwb plugin>``.GoalDist.scale:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         24.0            
  ============== =============================

  Description
    Scale for goal distance critic, overriding local default.

Debugging Visualizations Parameters
-----------------------------------

:``<dwb plugin>``.publish_evaluation:

  ==== =======
  Type Default
  ---- -------
  bool true      
  ==== =======

  Description
    Whether to publish the local plan evaluation.

:``<dwb plugin>``.publish_global_plan:

  ==== =======
  Type Default
  ---- -------
  bool true      
  ==== =======

  Description
    	Whether to publish the global plan.

:``<dwb plugin>``.publish_transformed_plan:

  ==== =======
  Type Default
  ---- -------
  bool true      
  ==== =======

  Description
    Whether to publish the global plan in the odometry frame.

:``<dwb plugin>``.publish_local_plan:

  ==== =======
  Type Default
  ---- -------
  bool true      
  ==== =======

  Description
    Whether to publish the local planner's plan.

:``<dwb plugin>``.publish_trajectories:

  ==== =======
  Type Default
  ---- -------
  bool true      
  ==== =======

  Description
    	Whether to publish debug trajectories.

:``<dwb plugin>``.publish_cost_grid_pc:

  ==== =======
  Type Default
  ---- -------
  bool false      
  ==== =======

  Description
    Whether to publish the cost grid.

:``<dwb plugin>``.marker_lifetime:

  ============== =======
  Type           Default
  -------------- -------
  double         0.1    
  ============== =======

  Description
    How long for the marker to remain.

kinematic_parameters
--------------------

:``<dwb plugin>``.max_vel_theta:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    Maximum angular velocity (rad/s).

:``<dwb plugin>``.min_speed_xy:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    Minimum translational speed (m/s).

:``<dwb plugin>``.max_speed_xy:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    Maximum translational speed (m/s).

:``<dwb plugin>``.min_speed_theta:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    Minimum angular speed (rad/s).

:``<dwb plugin>``.min_vel_x:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    	Minimum velocity X (m/s).

:``<dwb plugin>``.min_vel_y:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    Minimum velocity Y (m/s).

:``<dwb plugin>``.max_vel_x:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    	Maximum velocity X (m/s).

:``<dwb plugin>``.max_vel_y:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    Maximum velocity Y (m/s).

:``<dwb plugin>``.:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    .

:``<dwb plugin>``.acc_lim_x:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    	Maximum acceleration X (m/s^2).

:``<dwb plugin>``.acc_lim_y:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    Maximum acceleration Y (m/s^2).

:``<dwb plugin>``.acc_lim_theta:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    	Maximum acceleration rotation (rad/s^2).

:``<dwb plugin>``.decel_lim_x:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    Maximum deceleration X (m/s^2).

:``<dwb plugin>``.decel_lim_y:

  ====== =======
  Type   Default
  ------ -------
  double 0.0    
  ====== =======

  Description
    Maximum deceleration Y (m/s^2).

:``<dwb plugin>``.decel_lim_theta:

  ====== =======
  Type   Default
  ------ -------
  double 0.0
  ====== =======

  Description
    Maximum deceleration rotation (rad/s^2).

xy_theta_iterator
-----------------

:``<dwb plugin>``.vx_samples:

  ==== =======
  Type Default
  ---- -------
  int  20     
  ==== =======

  Description
    Number of velocity samples in the X velocity direction.

:``<dwb plugin>``.vy_samples:

  ==== =======
  Type Default
  ---- -------
  int  5     
  ==== =======

  Description
    Number of velocity samples in the Y velocity direction.

:``<dwb plugin>``.vtheta_samples:

  ==== =======
  Type Default
  ---- -------
  int  20     
  ==== =======

  Description
    Number of velocity samples in the angular directions.

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

DWB Plugins
***********

.. toctree::
  :maxdepth: 1

  dwb-plugins/simple_goal_checker.rst
  dwb-plugins/standard_traj_generator.rst
  dwb-plugins/limited_accel_generator.rst
  dwb-plugins/stopped_goal_checker.rst


Trajectory Critics
******************

.. toctree::
  :maxdepth: 1

  trajectory_critics/oscillation.rst
  trajectory_critics/base_obstacle.rst
  trajectory_critics/obstacle_footprint.rst
  trajectory_critics/prefer_forward.rst
  trajectory_critics/twirling.rst
  trajectory_critics/path_dist.rst
  trajectory_critics/goal_dist.rst
  trajectory_critics/goal_align.rst
  trajectory_critics/map_grid.rst
  trajectory_critics/path_align.rst
  trajectory_critics/rotate_to_goal.rst
