.. _configuring_mppic:

Model Predictive Path Integral Controller
#########################################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_mppi_controller


.. image:: images/mppi_demo.gif
    :width: 600px
    :alt: MPPI on Turtlebot3 demo
    :align: center

The MPPI Controller implements a `Model Predictive Path Integral Controller <https://ieeexplore.ieee.org/document/7487277>`_.
The new Nav2 MPPI Controller is a predictive controller - a successor to TEB and pure path tracking MPC controllers. It uses a sampling based approach to select optimal trajectories, optimizing between successive iterations. It contains plugin-based objective functions for customization and extension for various behaviors and behavioral attributes.

It works currently with Differential, Omnidirectional, and Ackermann robots.
This controller is measured to run at 50+ Hz on a modest Intel processor (4th gen i5).

The MPPI algorithm is an MPC variant that finds a control velocity for the robot using an iterative approach. Using the previous time step's best control solution and the robot's current state, a set of randomly sampled perturbations from a Gaussian distribution are applied. These noised controls are forward simulated to generate a set of trajectories within the robot's motion model.
Next, these trajectories are scored using a set of plugin-based critic functions to find the best trajectory in the batch. The output scores are used to set the best control with a soft max function.
This process is then repeated a number of times and returns a converged solution. This solution is then used as the basis of the next time step's initial control.

A powerful result of this work is the ability to utilize objective functions which are not require to be convex nor differentiable, providing greater designer flexibility in behavior.

See the package's ``README`` for more complete information.

MPPI Parameters
***************

:motion_model:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  string         "DiffDrive" 
  ============== ===========================

  Description
    The desired motion model to use for trajectory planning. Options are ``DiffDrive``, ``Omni``, or ``Ackermann``. Differential drive robots may use forward/reverse and angular velocities; Omni add in lateral motion; and Ackermann adds minimum curvature constraints.

:critics:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  string vector  N/A 
  ============== ===========================

  Description
    A vector of critic plugin functions to use, without ``mppi::critic::`` namespace which will be automatically added on loading.

:iteration_count:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1 
  ============== ===========================

  Description
    Iteration count in the MPPI algorithm. Recommended to remain as 1 and instead prefer larger batch sizes.

:batch_size:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1000 
  ============== ===========================

  Description
    Count of randomly sampled candidate trajectories from current optimal control sequence in a given iteration. 1000 @ 50 Hz or 2000 @ 30 Hz seems to produce good results.

:time_steps:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            56 
  ============== ===========================

  Description
    Number of time steps (points) in candidate trajectories

:model_dt:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.05 
  ============== ===========================

  Description
    Length of each time step's ``dt`` timestep, in seconds. ``time_steps * model_dt`` is the prediction horizon.

:vx_std:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.2 
  ============== ===========================

  Description
    Sampling standard deviation for Vx

:vy_std:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.2 
  ============== ===========================

  Description
    Sampling standard deviation for Vy

:wz_std:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.2 
  ============== ===========================

  Description
    Sampling standard deviation for Wz (angular velocity)

:vx_max:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.5 
  ============== ===========================

  Description
    Target maximum forward velocity (m/s).

:vy_max:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.5 
  ============== ===========================

  Description
    Target maximum lateral velocity, if using ``Omni`` motion model (m/s).

:vx_min:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         -0.35 
  ============== ===========================

  Description
    Maximum reverse velocity (m/s).

:wz_max:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         1.9 
  ============== ===========================

  Description
    Maximum rotational velocity (rad/s).

:temperature:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.3
  ============== ===========================

  Description
    Selectiveness of trajectories by their costs (The closer this value to 0, the "more" we take in consideration controls with less cost), 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories without cost consideration.

:gamma:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.015
  ============== ===========================

  Description
    A trade-off between smoothness (high) and low energy (low). This is a complex parameter that likely won't need to be changed from the default. See Section 3D-2 in "Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving" for detailed information. 

:visualize:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  bool           false
  ============== ===========================

  Description
    Whether to publish debuggin trajectories for visualization. This can slow down the controller substantially (e.g. 1000 batches of 56 size every 30hz is alot of data).

:retry_attempt_limit:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1
  ============== ===========================

  Description
    Number of attempts to find feasible trajectory on failure for soft-resets before reporting total failure.

:reset_period:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double            1.0
  ============== ===========================

  Description
    Required time of inactivity to reset optimizer  (only in Humble due to backport ABI policies).

Trajectory Visualization
------------------------

:trajectory_step:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            5
  ============== ===========================

  Description
    The step between trajectories to visualize to downsample candidate trajectory pool.

:time_step:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            3
  ============== ===========================

  Description
    The step between points on trajectories to visualize to downsample trajectory density. 

Path Handler
------------

:transform_tolerance:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.1
  ============== ===========================

  Description
    Time tolerance for data transformations with TF (s).

:prune_distance:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         1.5
  ============== ===========================

  Description
    Distance ahead of nearest point on path to robot to prune path to (m).

:max_robot_pose_search_dist:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         Costmap size / 2
  ============== ===========================

  Description
    Max integrated distance ahead of robot pose to search for nearest path point in case of path looping.


Ackermann Motion Model
----------------------

:min_turning_r:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.2
  ============== ===========================

  Description
    The minimum turning radius possible for the vehicle platform (m).


Constraint Critic
-----------------

:cost_weight:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         4.0
  ============== ===========================

  Description
    Weight to apply to critic term.

:cost_power:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1
  ============== ===========================

  Description
    Power order to apply to term. 

Goal Angle Critic
-----------------

:cost_weight:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         3.0
  ============== ===========================

  Description
    Weight to apply to critic term.

:cost_power:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1
  ============== ===========================

  Description
    Power order to apply to term. 

:threshold_to_consider:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.4
  ============== ===========================

  Description
    Minimal distance (m) between robot and goal above which angle goal cost considered.

Goal Critic
-----------

:cost_weight:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         5.0
  ============== ===========================

  Description
    Weight to apply to critic term.

:cost_power:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1
  ============== ===========================

  Description
    Power order to apply to term. 

:threshold_to_consider:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         1.0
  ============== ===========================

  Description
    Minimal distance (m) between robot and goal above which goal distance cost considered.

Obstacles Critic
----------------

:critical_weight:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         20.0
  ============== ===========================

  Description
    Weight to apply to critic for near collisions closer than ``collision_margin_distance`` to prevent near collisions **only** as a method of virtually inflating the footprint. This should not be used to generally influence obstacle avoidance away from critical collisions.

:repulsion_weight:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         1.5
  ============== ===========================

  Description
    Weight to apply to critic for generally preferring routes in lower cost space. This is separated from the critical term to allow for fine tuning of obstacle behaviors with path alignment for dynamic scenes without impacting actions which may directly lead to near-collisions. This is applied within the ``inflation_radius`` distance from obstacles.

:cost_power:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1
  ============== ===========================

  Description
    Power order to apply to term.

:consider_footprint:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  bool           false
  ============== ===========================

  Description
    Whether to use point cost (if robot is circular or low compute power) or compute SE2 footprint cost.

:collision_cost:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         10000.0
  ============== ===========================

  Description
    Cost to apply to a true collision in a trajectory.

:collision_margin_distance:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.10
  ============== ===========================

  Description
    Margin distance (m) from collision to apply severe penalty, similar to footprint inflation. Between 0.05-0.2 is reasonable. Note that it will highly influence the controller not to enter spaces more confined than this, so ensure this parameter is set lower than the narrowest you expect the robot to need to traverse through.

:near_goal_distance:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.50
  ============== ===========================

  Description
    Distance (m) near goal to stop applying preferential obstacle term to allow robot to smoothly converge to goal pose in close proximity to obstacles.

:cost_scaling_factor:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         10.0
  ============== ===========================

  Description
    Exponential decay factor across inflation radius. This should be the same as for your inflation layer (Humble only)

:inflation_radius:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.55
  ============== ===========================

  Description
    Radius to inflate costmap around lethal obstacles. This should be the same as for your inflation layer (Humble only)

Path Align Critic
-----------------

:cost_weight:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         10.0
  ============== ===========================

  Description
    Weight to apply to critic term.

:cost_power:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1
  ============== ===========================

  Description
    Power order to apply to term. 

:threshold_to_consider:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.4
  ============== ===========================

  Description
    Distance (m) between robot and goal to **stop** considering path alignment and allow goal critics to take over.

:offset_from_furthest:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            20
  ============== ===========================

  Description
    Checks that the candidate trajectories are sufficiently far along their way tracking the path to apply the alignment critic. This ensures that path alignment is only considered when actually tracking the path, preventing awkward initialization motions preventing the robot from leaving the path to achieve the appropriate heading.

:max_path_occupancy_ratio:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.07
  ============== ===========================

  Description
    Maximum proportion of the path that can be occupied before this critic is not considered to allow the obstacle and path follow critics to avoid obstacles while following the path's intent in presence of dynamic objects in the scene. Between 0-1 for 0-100%.

Path Angle Critic
-----------------

:cost_weight:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         2.0
  ============== ===========================

  Description
    Weight to apply to critic term.

:cost_power:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1
  ============== ===========================

  Description
    Power order to apply to term.

:threshold_to_consider:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.4
  ============== ===========================

  Description
    Distance (m) between robot and goal to **stop** considering path angles and allow goal critics to take over.

:offset_from_furthest:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            20
  ============== ===========================

  Description
    Number of path points after furthest one any trajectory achieves to compute path angle relative to.

:max_angle_to_furthest:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         1.2
  ============== ===========================

  Description
    Angular distance (rad) between robot and goal above which path angle cost starts being considered

Path Follow Critic
------------------

:cost_weight:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         5.0
  ============== ===========================

  Description
    Weight to apply to critic term.

:cost_power:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1
  ============== ===========================

  Description
    Power order to apply to term.

:threshold_to_consider:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.4
  ============== ===========================

  Description
    Distance (m) between robot and goal to **stop** considering path following and allow goal critics to take over.

:offset_from_furthest:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            6
  ============== ===========================

  Description
    Number of path points after furthest one any trajectory achieves to drive path tracking relative to.

Prefer Forward Critic
---------------------

:cost_weight:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         5.0
  ============== ===========================

  Description
    Weight to apply to critic term.

:cost_power:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1
  ============== ===========================

  Description
    Power order to apply to term.

:threshold_to_consider:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.4
  ============== ===========================

  Description
    Distance (m) between robot and goal to **stop** considering preferring forward and allow goal critics to take over.

Twirling Critic
---------------

:cost_weight:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         10.0
  ============== ===========================

  Description
    Weight to apply to critic term.

:cost_power:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1
  ============== ===========================

  Description
    Power order to apply to term.

Example
*******
.. code-block:: yaml

    controller_server:
      ros__parameters:
        controller_frequency: 30.0
        FollowPath:
          plugin: "nav2_mppi_controller::MPPIController"
          time_steps: 56
          model_dt: 0.05
          batch_size: 2000
          vx_std: 0.2
          vy_std: 0.2
          wz_std: 0.4
          vx_max: 0.5
          vx_min: -0.35
          vy_max: 0.5
          wz_max: 1.9
          iteration_count: 1
          prune_distance: 1.7
          transform_tolerance: 0.1
          temperature: 0.3
          gamma: 0.015
          motion_model: "DiffDrive"
          visualize: false
          reset_period: 1.0 # (only in Humble)
          TrajectoryVisualizer:
            trajectory_step: 5
            time_step: 3
          AckermannConstrains:
            min_turning_r: 0.2
          critics: ["ConstraintCritic", "ObstaclesCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]
          ConstraintCritic:
            enabled: true
            cost_power: 1
            cost_weight: 4.0
          GoalCritic:
            enabled: true
            cost_power: 1
            cost_weight: 5.0
            threshold_to_consider: 1.0
          GoalAngleCritic:
            enabled: true
            cost_power: 1
            cost_weight: 3.0
            threshold_to_consider: 0.4
          PreferForwardCritic:
            enabled: true
            cost_power: 1
            cost_weight: 5.0
            threshold_to_consider: 0.4
          ObstaclesCritic:
            enabled: true
            cost_power: 1
            repulsion_weight: 1.5
            critical_weight: 20.0
            consider_footprint: false
            collision_cost: 10000.0
            collision_margin_distance: 0.1
            near_goal_distance: 0.5
            inflation_radius: 0.55 # (only in Humble)
            cost_scaling_factor: 10.0 # (only in Humble)
          PathAlignCritic:
            enabled: true
            cost_power: 1
            cost_weight: 14.0
            max_path_occupancy_ratio: 0.05
            trajectory_point_step: 3
            threshold_to_consider: 0.40
            offset_from_furthest: 20
          PathFollowCritic:
            enabled: true
            cost_power: 1
            cost_weight: 5.0
            offset_from_furthest: 5
            threshold_to_consider: 0.6
          PathAngleCritic:
            enabled: true
            cost_power: 1
            cost_weight: 2.0
            offset_from_furthest: 4
            threshold_to_consider: 0.40
            max_angle_to_furthest: 1.0
          # TwirlingCritic:
          #   enabled: true
          #   twirling_cost_power: 1
          #   twirling_cost_weight: 10.0


Notes to Users
**************

General Words of Wisdom
-----------------------

The ``model_dt`` parameter generally should be set to the duration of your control frequency. So if your control frequency is 20hz, this should be ``0.05``. However, you may also set it lower **but not larger**.

Visualization of the trajectories using ``visualize`` uses compute resources to back out trajectories for visualization and therefore slows compute time. It is not suggested that this parameter is set to ``true`` during a deployed use, but is a useful debug instrument while tuning the system, but use sparingly. Visualizing 2000 batches @ 56 points at 30 hz is *alot*.

The most common parameters you might want to start off changing are the velocity profiles (``vx_max``, ``vx_min``, ``wz_max``, and ``vy_max`` if holonomic) and the ``motion_model`` to correspond to your vehicle. Its wise to consider the ``prune_distance`` of the path plan in proportion to your maximum velocity and prediction horizon. The only deeper parameter that will likely need to be adjusted for your particular settings is the Obstacle critics' ``repulsion_weight`` since the tuning of this is proprtional to your inflation layer's radius. Higher radii should correspond to reduced ``repulsion_weight`` due to the penalty formation (e.g. ``inflation_radius - min_dist_to_obstacle``). If this penalty is too high, the robot will slow significantly when entering cost-space from non-cost space or jitter in narrow corridors. It is noteworthy, but likely not necessary to be changed, that the Obstacle critic may use the full footprint information if ``consider_footprint = true``, though comes at an increased compute cost.

Otherwise, the parameters have been closely pre-tuned by your friendly neighborhood navigator to give you a decent starting point that hopefully you only need to retune for your specific desired behavior lightly (if at all). Varying costmap parameters or maximum speeds are the actions which require the most attention, as described below:

Prediction Horizon, Costmap Sizing, and Offsets
-----------------------------------------------

As this is a predictive planner, there is some relationship between maximum speed, prediction times, and costmap size that users should keep in mind while tuning for their application. If a controller server costmap is set to 3.0m in size, that means that with the robot in the center, there is 1.5m of information on either side of the robot. When your prediction horizon (``time_steps * model_dt``) at maximum speed (``vx_max``) is larger than this, then your robot will be artificially limited in its maximum speeds and behavior by the costmap limitation. For example, if you predict forward 3 seconds (60 steps @ 0.05s per step) at 0.5m/s maximum speed, the **minimum** required costmap radius is 1.5m - or 3m total width.

The same applies to the Path Follow and Align offsets from furthest. In the same example if the furthest point we can consider is already at the edge of the costmap, then further offsets are thresholded because they're unusable. So its important while selecting these parameters to make sure that the theoretical offsets can exist on the costmap settings selected with the maximum prediction horizon and velocities desired.

The Path Follow critic cannot drive velocities greater than the projectable distance of that velocity on the available path on the rolling costmap. The Path Align critic `offset_from_furthest` represents the number of path points a trajectory passes through while tracking the path. If this is set either absurdly low (e.g. 5) it can trigger when a robot is simply trying to start path tracking causing some suboptimal behaviors and local minima while starting a task. If it is set absurdly high (e.g. 50) relative to the path resolution and costmap size, then the critic may never trigger or only do so when at full-speed. A balance here is wise. A selection of this value to be ~30% of the maximum velocity distance projected is good (e.g. if a planner produces points every 2.5cm, 60 can fit on the 1.5m local costmap radius. If the max speed is 0.5m/s with a 3s prediction time, then 20 points represents 33% of the maximum speed projected over the prediction horizon onto the path). When in doubt, ``prediction_horizon_s * max_speed / path_resolution / 3.0`` is a good baseline.

Obstacle, Inflation Layer, and Path Following
---------------------------------------------

There also exists a relationship between the costmap configurations and the Obstacle critic configurations. If the Obstacle critic is not well tuned with the costmap parameters (inflation radius, scale) it can cause the robot to wobble significantly as it attempts to take finitely lower-cost trajectories with a slightly lower cost in exchange for jerky motion. It may also perform awkward maneuvers when in free-space to try to maximize time in a small pocket of 0-cost over a more natural motion which involves moving into some low-costed region. Finally, it may generally refuse to go into costed space at all when starting in a free 0-cost space if the gain is set disproportionately higher than the Path Follow scoring to encourage the robot to move along the path. This is due to the critic cost of staying in free space becoming more attractive than entering even lightly costed space in exchange for progression along the task. 

Thus, care should be taken to select weights of the obstacle critic in conjunction with the costmap inflation radius and scale so that a robot does not have such issues. How I (Steve, your friendly neighborhood navigator) tuned this was to first create the appropriate obstacle critic behavior desirable in conjunction with the inflation layer parameters. Its worth noting that the Obstacle critic converts the cost into a distance from obstacles, so the nature of the distribution of costs in the inflation isn't overly significant. However, the inflation radius and the scale will define the cost at the end of the distribution where free-space meets the lowest cost value within the radius. So testing for quality behavior when going over that threshold should be considered.

As you increase or decrease your weights on the Obstacle, you may notice the aforementioned behaviors (e.g. won't overcome free to non-free threshold). To overcome them, increase the FollowPath critic cost to increase the desire for the trajectory planner to continue moving towards the goal. Make sure to not overshoot this though, keep them balanced. A desirable outcome is smooth motion roughly in the center of spaces without significant close interactions with obstacles. It shouldn't be perfectly following a path yet nor should the output velocity be wobbling jaggedly.

Once you have your obstacle avoidance behavior tuned and matched with an appropriate path following penalty, tune the Path Align critic to align with the path. If you design exact-path-alignment behavior, its possible to skip the obstacle critic step as highly tuning the system to follow the path will give it less ability to deviate to avoid obstacles (though it'll slow and stop). Tuning the critic weight for the Obstacle critic high will do the job to avoid near-collisions but the repulsion weight is largely unnecessary to you. For others wanting more dynamic behavior, it *can* be beneficial to slowly lower the weight on the obstacle critic to give the path alignment critic some more room to work. If your path was generated with a cost-aware planner (like all provided by Nav2) and providing paths sufficiently far from obstacles for your satisfaction, the impact of a slightly reduced Obstacle critic with a Path Alignment critic will do you well. Not over-weighting the path align critic will allow the robot to  deviate from the path to get around dynamic obstacles in the scene or other obstacles not previous considered during path planning. It is subjective as to the best behavior for your application, but it has been shown that MPPI can be an exact path tracker and/or avoid dynamic obstacles very fluidly and everywhere in between. The defaults provided are in the generally right regime for a balanced initial trade-off. 
