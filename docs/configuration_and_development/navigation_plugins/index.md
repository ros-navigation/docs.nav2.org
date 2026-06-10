# Navigation Plugins { #navigation-plugins }

There are a number of plugin interfaces for users to create their own custom applications or algorithms with.
Namely, the costmap layer, planner, controller, behavior tree, and behavior plugins.
A list of all known plugins are listed here below for ROS 2 Navigation.
If you know of a plugin, or you have created a new plugin, please consider submitting a pull request with that information.

This file can be found and edited under `docs/configuration_and_development/navigation_plugins/index.md`.
For tutorials on creating your own plugins, please see 
[Writing a New Costmap2D Plugin][writing-a-new-costmap-2d-plugin], 
[Writing a New Behavior Tree Plugin][writing-a-new-behavior-tree-plugin], 
[Writing a New Controller Plugin][writing-a-new-controller-plugin], 
[Writing a New Planner Plugin][writing-a-new-planner-plugin], 
[Writing a New Behavior Plugin][writing-a-new-behavior-plugin], or 
[Writing a New Navigator Plugin][writing-a-new-navigator-plugin].

## Behavior-Tree Navigators

<div class="center-table" markdown>

| Plugin Name                       | Creator        | Description                                                           |
|-----------------------------------|----------------|-----------------------------------------------------------------------|
| [NavigateToPoseNavigator][]       | Steve Macenski | Point-to-point navigation via a<br>behavior tree action server        |
| [NavigateThroughPosesNavigator][] | Steve Macenski | Point-through-points navigation<br>via a behavior tree action server  |
| [CoverageNavigator][]             | Steve Macenski | Complete coverage navigation<br>(Cartesian or GPS) via a BTs          |

</div>

[NavigateToPoseNavigator]: https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator/src/navigators
[NavigateThroughPosesNavigator]: https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator/src/navigators
[CoverageNavigator]: https://github.com/open-navigation/opennav_coverage/tree/main/opennav_coverage_navigator

## Costmap Layers

<div class="center-table" markdown>

| Plugin Name                            | Creator                    | Description                                                                                                                                                                |
|----------------------------------------|----------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Voxel Layer][]                        | Eitan Marder-Eppstein      | Maintains persistent<br>3D voxel layer using depth and<br>laser sensor readings and<br>raycasting to clear free space                                                      |
| [Range Layer][]                        | David Lu                   | Uses a probabilistic model to<br>put data from sensors that<br>publish range msgs on the costmap                                                                           |
| [Static Layer][]                       | Eitan Marder-Eppstein      | Gets static `map` and loads<br>occupancy information into<br>costmap                                                                                                       |
| [Inflation Layer][]                    | Eitan Marder-Eppstein      | Inflates lethal obstacles in<br>costmap with exponential decay                                                                                                             |
| [Obstacle Layer][]                     | Eitan Marder-Eppstein      | Maintains persistent 2D costmap<br>from 2D laser scans with<br>raycasting to clear free space                                                                              |
| [Spatio-Temporal Voxel Layer][]        | Steve Macenski             | Maintains temporal 3D sparse<br>volumetric voxel grid with decay<br>through sensor models                                                                                  |
| [Non-Persistent Voxel Layer][]         | Steve Macenski             | Maintains 3D occupancy grid<br>consisting only of the most<br>sets of measurements                                                                                         |
| [Denoise Layer][]                      | Andrey Ryzhikov            | Filters noise-induced<br>standalone obstacles or small<br>obstacles groups                                                                                                 |
| [Plugin Container Layer][]             | Alexander Yuen             | Combines the different costmap<br>layers specified under this<br>layer in order populate the same<br>costmap with different isolated<br>combinations of costmap layers     |
| [Ground Consistency Layer][]           | Muhammad Haider Khan Lodhi | Height-aware costmap layer using<br>3D ground segmentation. Pair<br>with Inflation Layer for terrain-aware navigation.                                                     |

</div>

[Voxel Layer]: https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d/plugins/voxel_layer.cpp
[Range Layer]: https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d/plugins/range_sensor_layer.cpp
[Static Layer]: https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d/plugins/static_layer.cpp
[Inflation Layer]: https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d/plugins/inflation_layer.cpp
[Obstacle Layer]: https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d/plugins/obstacle_layer.cpp
[Spatio-Temporal Voxel Layer]: https://github.com/SteveMacenski/spatio_temporal_voxel_layer/
[Non-Persistent Voxel Layer]: https://github.com/SteveMacenski/nonpersistent_voxel_layer
[Denoise Layer]: https://github.com/ryzhikovas/navigation2/tree/feature-costmap2d-denoise/nav2_costmap_2d/plugins/denoise_layer.cpp
[Plugin Container Layer]: https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d/plugins/plugin_container_layer.cpp
[Ground Consistency Layer]: https://github.com/dfki-ric/nav2_ground_consistency_costmap_plugin

## Costmap Filters { #navigation-plugins-costmap-filters }

<div class="center-table" markdown>

| Plugin Name          | Creator           | Description                                                        |
|----------------------|-------------------|--------------------------------------------------------------------|
| [Keepout Filter][]   | Alexey Merzlyakov | Maintains keep-out/safety zones<br>and preferred lanes for moving  |
| [Speed Filter][]     | Alexey Merzlyakov | Limits maximum velocity of robot<br>in speed restriction areas     |
| [Binary Filter][]    | Alexey Merzlyakov | Enables binary (boolean) mask<br>behavior to trigger actions       |

</div>

[Keepout Filter]: https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp
[Speed Filter]: https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d/plugins/costmap_filters/speed_filter.cpp
[Binary Filter]: https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d/plugins/costmap_filters/binary_filter.cpp

## Controllers

<div class="center-table" markdown>

| Plugin Name                               | Creator                             | Description                                                                                            | Drivetrain support                                           |
|-------------------------------------------|-------------------------------------|--------------------------------------------------------------------------------------------------------|--------------------------------------------------------------|
| [DWB Controller][]                        | David Lu!!                          | A highly configurable  DWA<br>implementation with plugin<br>interfaces                                 | Differential,<br>Omnidirectional,<br>Legged                  |
| [TEB Controller][]                        | Christoph Rösmann                   | A MPC-like controller suitable<br>for ackermann, differential, and<br>holonomic robots.                | **Ackermann**, Legged,<br>Omnidirectional,<br>Differential   |
| [Regulated Pure Pursuit][]                | Steve Macenski                      | A service / industrial robot<br>variation on the pure pursuit<br>algorithm with adaptive features.     | **Ackermann**, Legged,<br>Differential                       |
| [MPPI Controller][]                       | Steve Macenski<br>Aleksei Budyakov  | A predictive MPC controller with<br>modular & custom cost functions<br>that can accomplish many tasks. | Differential, Omni,<br>**Ackermann**                         |
| [Rotation Shim Controller][]              | Steve Macenski                      | A "shim" controller to rotate<br>to path heading before passing<br>to main controller for  tracking.   | Differential, Omni,<br>model rotate in place                 |
| [Graceful Controller][]                   | Alberto Tudela                      | A controller based on a<br>pose-following control law to<br>generate smooth trajectories.              | Differential, Omni,<br>Legged                                |
| [Vector Pursuit Controller][]             | Black Coffee Robotics               | A controller based on the vector<br>pursuit algorithm useful for<br>high speed accurate path tracking. | Differential,<br>Ackermann, Legged                           |

</div>

[DWB Controller]: https://github.com/ros-navigation/navigation2/tree/main/nav2_dwb_controller
[TEB Controller]: https://github.com/rst-tu-dortmund/teb_local_planner
[Regulated Pure Pursuit]: https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller
[MPPI Controller]: https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller
[Rotation Shim Controller]: https://github.com/ros-navigation/navigation2/tree/main/nav2_rotation_shim_controller
[Graceful Controller]: https://github.com/ros-navigation/navigation2/tree/main/nav2_graceful_controller
[Vector Pursuit Controller]: https://github.com/blackcoffeerobotics/vector_pursuit_controller

## Planners

<div class="center-table" markdown>

| Plugin Name                                         | Creator                               | Description                                                                                                                                                                                                                                                          | Drivetrain support                                                                   |
|-----------------------------------------------------|---------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------|
| [NavFn Planner][]                                   | Eitan Marder-Eppstein & Kurt Konolige | A navigation function<br>using A\* or Dijkstras<br>expansion, assumes 2D<br>holonomic particle                                                                                                                                                                       | Differential,<br>Omnidirectional,<br>Legged                                          |
| [SmacPlannerHybrid][] <br> (formerly *SmacPlanner*) | Steve Macenski                        | A SE2 Hybrid-A\*<br>implementation using either<br>Dubin or Reeds-shepp motion<br>models with smoother and<br>multi-resolution query.<br>Cars, car-like, and<br>ackermann vehicles.<br>Kinematically feasible.                                                       | **Ackermann**,<br>Differential,<br>Omnidirectional,<br>Legged                        |
| [SmacPlanner2D][]                                   | Steve Macenski                        | A 2D A\* implementation<br>Using either 4 or 8<br>connected neighborhoods<br>with smoother and<br>multi-resolution query                                                                                                                                             | Differential,<br>Omnidirectional,<br>Legged                                          |
| [SmacPlannerLattice][]                              | Steve Macenski                        | An implementation of State<br>Lattice Planner using<br>pre-generated<br>minimum control sets for kinematically<br>feasible planning with any<br>type of vehicle imaginable.<br>Includes generator script for<br>Ackermann, diff, omni, and<br>legged robots.         | Differential,<br>Omnidirectional,<br>Ackermann,<br>Legged,<br>Arbitrary / Custom     |
| [ThetaStarPlanner][]                                | Anshumaan Singh                       | An implementation of Theta\*<br>using either 4 or 8<br>connected neighborhoods,<br>assumes the robot as a<br>2D holonomic particle                                                                                                                                   | Differential,<br>Omnidirectional                                                     |

</div>

[NavFn Planner]: https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner
[SmacPlannerHybrid]: https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner
[SmacPlanner2D]: https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner
[SmacPlannerLattice]: https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner
[ThetaStarPlanner]: https://github.com/ros-navigation/navigation2/tree/main/nav2_theta_star_planner

## Smoothers

<div class="center-table" markdown>

| Plugin Name                          | Creator                          | Description                                                                                                                                                                               |
|--------------------------------------|----------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Simple Smoother][]                  | Steve Macenski                   | A simple path smoother for<br>infeasible (e.g. 2D)<br>planners                                                                                                                            |
| [Constrained Smoother][]             | Matej Vargovcik & Steve Macenski | A path smoother using a<br>constraints problem solver<br>to optimize various criteria<br>such as smoothness or<br>distance from obstacles,<br>maintaining minimum turning<br>radius       |
| [Savitzky-Golay Smoother][]          | Steve Macenski                   | A path smoother using a<br>Savitzky-Golay filter<br>to smooth the path via<br>digital signal processing<br>to remove noise from the<br>path.                                              |

</div>

[Simple Smoother]: https://github.com/ros-navigation/navigation2/tree/main/nav2_smoother
[Constrained Smoother]: https://github.com/ros-navigation/navigation2/tree/main/nav2_constrained_smoother
[Savitzky-Golay Smoother]: https://github.com/ros-navigation/navigation2/tree/main/nav2_smoother

## Behaviors

<div class="center-table" markdown>

| Plugin Name                       | Creator               | Description                                                                                                                            |
|-----------------------------------|-----------------------|----------------------------------------------------------------------------------------------------------------------------------------|
| [Clear Costmap][]                 | Eitan Marder-Eppstein | A service to clear the given<br>costmap in case of incorrect<br>perception or robot is stuck                                           |
| [Spin][]                          | Steve Macenski        | Rotate behavior of configurable<br>angles to clear out free space<br>and nudge robot out of potential<br>local failures                |
| [Back Up][]                       | Brian Wilcox          | Back up behavior of configurable<br>distance to back out of a<br>situation where the robot is<br>stuck                                 |
| [Wait][]                          | Steve Macenski        | Wait behavior with configurable<br>time to wait in case of time<br>based obstacle like human traffic<br>or getting more sensor data    |
| [Drive On Heading][]              | Joshua Wallace        | Drive on heading behavior with<br>configurable distance to drive                                                                       |
| [Assisted Teleop][]               | Joshua Wallace        | AssistedTeleop behavior that<br>scales teleop commands to<br>prevent collisions.                                                       |

</div>

[Clear Costmap]: https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/src/clear_costmap_service.cpp
[Spin]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors/plugins
[Back Up]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors/plugins
[Wait]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors/plugins
[Drive On Heading]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors/plugins
[Assisted Teleop]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors/plugins

## Waypoint Task Executors

<div class="center-table" markdown>

| Plugin Name          | Creator        | Description                                                                           |
|----------------------|----------------|---------------------------------------------------------------------------------------|
| [WaitAtWaypoint][]   | Fetullah Atas  | A plugin to execute a wait<br>behavior  on<br>waypoint arrivals.                      |
| [PhotoAtWaypoint][]  | Fetullah Atas  | A plugin to take and save photos<br>to specified directory on<br>waypoint arrivals.   |
| [InputAtWaypoint][]  | Steve Macenski | A plugin to wait for user input<br>before moving onto the next<br>waypoint.           |

</div>

[WaitAtWaypoint]: https://github.com/ros-navigation/navigation2/tree/main/nav2_waypoint_follower/plugins/wait_at_waypoint.cpp
[PhotoAtWaypoint]: https://github.com/ros-navigation/navigation2/tree/main/nav2_waypoint_follower/plugins/photo_at_waypoint.cpp
[InputAtWaypoint]: https://github.com/ros-navigation/navigation2/tree/main/nav2_waypoint_follower/plugins/input_at_waypoint.cpp

## Goal Checkers

<div class="center-table" markdown>

| Plugin Name                        | Creator                       | Description                                                                                                                          |
|------------------------------------|-------------------------------|--------------------------------------------------------------------------------------------------------------------------------------|
| [SimpleGoalChecker][]              | David Lu!!                    | A plugin check whether robot<br>is within translational distance<br>and rotational distance of goal.                                 |
| [StoppedGoalChecker][]             | David Lu!!                    | A plugin check whether robot<br>is within translational distance,<br>rotational distance of goal,<br>and velocity threshold.         |
| [PositionGoalChecker][]            | Prabhav Saxena                | A plugin check whether robot<br>is within translational distance<br>of goal, without requiring<br>rotational convergence.            |

</div>

[SimpleGoalChecker]: https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/plugins/simple_goal_checker.cpp
[StoppedGoalChecker]: https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/plugins/stopped_goal_checker.cpp
[PositionGoalChecker]: https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/plugins/position_goal_checker.cpp

## Progress Checkers

<div class="center-table" markdown>

| Plugin Name                | Creator         | Description                                                                                                                                  |
|----------------------------|-----------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| [SimpleProgressChecker][]  | David Lu!!      | A plugin to check whether the<br>robot was able to move a minimum<br>distance in a given time to<br>make progress towards a goal             |
| [PoseProgressChecker][]    | Guillaume Doisy | A plugin to check whether the<br>robot was able to move a minimum<br>distance or angle in a given time<br>to make progress towards a goal    |

</div>

[SimpleProgressChecker]: https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/plugins/simple_progress_checker.cpp
[PoseProgressChecker]: https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/plugins/pose_progress_checker.cpp

## Behavior Tree Nodes

<div class="center-table" markdown>

| Action Plugin Name                                                     | Creator                        | Description                                                                                                                |
|------------------------------------------------------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------------------------|
| [Back Up Action][]                                                     | Michael Jeronimo               | Calls backup behavior action                                                                                               |
| [Drive On Heading Action][]                                            | Joshua Wallace                 | Calls drive on heading behavior action                                                                                     |
| [Assisted Teleop Action][]                                             | Joshua Wallace                 | Calls assisted teleop behavior action                                                                                      |
| [Clear Entire Costmap Service][]                                       | Carl Delsey                    | Calls clear entire costmap service                                                                                         |
| [Clear Costmap Except Region Service][]                                | Guillaume Doisy                | Calls clear costmap except region service                                                                                  |
| [Clear Costmap Around Robot Service][]                                 | Guillaume Doisy                | Calls clear costmap around robot service                                                                                   |
| [Compute Path to Pose Action][]                                        | Michael Jeronimo               | Calls Nav2 planner server                                                                                                  |
| [Smooth Path Action][]                                                 | Matej Vargovcik                | Calls Nav2 smoother server                                                                                                 |
| [Follow Path Action][]                                                 | Michael Jeronimo               | Calls Nav2 controller server                                                                                               |
| [Navigate to Pose Action][]                                            | Michael Jeronimo               | BT Node for other<br>BehaviorTree.CPP BTs to call<br>Navigation2 as a subtree action                                       |
| [Reinitialize Global Localization Service][]                           | Carl Delsey                    | Reinitialize AMCL to a new pose                                                                                            |
| [Spin Action][]                                                        | Carl Delsey                    | Calls spin behavior action                                                                                                 |
| [Wait Action][]                                                        | Steve Macenski                 | Calls wait behavior action                                                                                                 |
| [Truncate Path][]                                                      | Francisco Martín               | Modifies a path making it shorter                                                                                          |
| [Truncate Path Local][]                                                | Matej Vargovcik                | Extracts a path section around robot                                                                                       |
| [Planner Selector][]                                                   | Pablo Iñigo Blasco             | Selects the global planner based on a<br>topic input, otherwises uses a default<br>planner id                              |
| [Controller Selector][]                                                | Pablo Iñigo Blasco             | Selects the controller based on a<br>topic input, otherwises uses a default<br>controller id                               |
| [Goal Checker Selector][]                                              | Pablo Iñigo Blasco             | Selects the goal checker based on a<br>topic input, otherwises uses a default<br>goal checker id                           |
| [Smoother Selector][]                                                  | Owen Hooper                    | Selects the smoother based on a<br>topic input, otherwises uses a default<br>smoother id                                   |
| [Progress Checker Selector][]                                          | Steve Macenski                 | Selects the progress checker based on a<br>topic input, otherwises uses a default<br>progress checker id                   |
| [Navigate Through Poses][]                                             | Steve Macenski                 | BT Node for other BehaviorTree.CPP BTs<br>to call Nav2's NavThroughPoses action                                            |
| [Remove Passed Goals][]                                                | Steve Macenski                 | Removes goal poses passed or within a<br>tolerance for culling old viapoints from<br>path re-planning                      |
| [Compute Path Through Poses][]                                         | Steve Macenski                 | Computes a path through a set of poses<br>rather than a single end goal pose<br>using the planner plugin specified         |
| [Compute Route][]                                                      | Steve Macenski                 | Computes a Route through a navigation<br>graph and returns both a dense path and<br>set of sparse route nodes and edges.   |
| [Compute And Track Route][]                                            | Steve Macenski                 | Computes a Route as above, but also<br>actively tracks progress and triggers<br>route contextual semantic operations.      |
| [Cancel Control Action][]                                              | Pradheep Padmanabhan           | Cancels Nav2 controller server                                                                                             |
| [Cancel BackUp Action][]                                               | Pradheep Padmanabhan           | Cancels backup behavior action                                                                                             |
| [Cancel Spin Action][]                                                 | Pradheep Padmanabhan           | Cancels spin behavior action                                                                                               |
| [Cancel Wait Action][]                                                 | Pradheep Padmanabhan           | Cancels wait behavior action                                                                                               |
| [Cancel Route Action][]                                                | Steve Macenski                 | Cancels ComputeAndTrackRoute action                                                                                        |
| [Cancel Drive on Heading Action][]                                     | Joshua Wallace                 | Cancels drive on heading behavior action                                                                                   |
| [Cancel Assisted Teleop Action][]                                      | Joshua Wallace                 | Cancels assisted teleop behavior action                                                                                    |
| [Cancel Complete Coverage Action][]                                    | Steve Macenski                 | Cancels compute complete coverage                                                                                          |
| [Compute Complete Coverage Path Action][]                              | Steve Macenski                 | Calls coverage planner server                                                                                              |
| [Get Pose From Path Action][]                                          | Marc Morcos                    | Extracts a pose from a path                                                                                                |
| [Dock Robot Action][]                                                  | Steve Macenski                 | Calls dock robot action                                                                                                    |
| [Undock Robot Action][]                                                | Steve Macenski                 | Calls undock robot action                                                                                                  |
| [Concatenate Paths Action][]                                           | Steve Macenski                 | Concatenates 2 paths together                                                                                              |
| [Get Current Pose Action][]                                            | Steve Macenski                 | Gets current pose to the blackboard                                                                                        |

</div>

[Back Up Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/back_up_action.cpp
[Drive On Heading Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/drive_on_heading_action.cpp
[Assisted Teleop Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/assisted_teleop_action.cpp
[Clear Entire Costmap Service]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
[Clear Costmap Except Region Service]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
[Clear Costmap Around Robot Service]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
[Compute Path to Pose Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp
[Smooth Path Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/smooth_path_action.cpp
[Follow Path Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/follow_path_action.cpp
[Navigate to Pose Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/navigate_to_pose_action.cpp
[Reinitialize Global Localization Service]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/reinitialize_global_localization_service.cpp
[Spin Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/spin_action.cpp
[Wait Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/wait_action.cpp
[Truncate Path]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/truncate_path_action.cpp
[Truncate Path Local]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/truncate_path_local_action.cpp
[Planner Selector]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/planner_selector_node.cpp
[Controller Selector]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/controller_selector_node.cpp
[Goal Checker Selector]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/goal_checker_selector_node.cpp
[Smoother Selector]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/smoother_selector_node.cpp
[Progress Checker Selector]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/progress_checker_selector_node.cpp
[Navigate Through Poses]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/navigate_through_poses_action.cpp
[Remove Passed Goals]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/remove_passed_goals_action.cpp
[Compute Path Through Poses]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/compute_path_through_poses_action.cpp
[Compute Route]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/compute_route_action.cpp
[Compute And Track Route]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/compute_and_track_route_action.cpp
[Cancel Control Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/controller_cancel_node.cpp
[Cancel BackUp Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/back_up_cancel_node.cpp
[Cancel Spin Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/spin_cancel_node.cpp
[Cancel Wait Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/wait_cancel_node.cpp
[Cancel Route Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/compute_and_track_route_cancel_node.cpp
[Cancel Drive on Heading Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/drive_on_heading_cancel_node.cpp
[Cancel Assisted Teleop Action]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/action/assisted_teleop_cancel_node.cpp
[Cancel Complete Coverage Action]: https://github.com/open-navigation/opennav_coverage/blob/main/opennav_coverage_bt/src/cancel_complete_coverage_path.cpp
[Compute Complete Coverage Path Action]: https://github.com/open-navigation/opennav_coverage/blob/main/opennav_coverage_bt/src/compute_complete_coverage_path.cpp
[Get Pose From Path Action]: https://github.com/ros-navigation/navigation2/blob/main/nav2_behavior_tree/plugins/action/get_pose_from_path_action.cpp
[Dock Robot Action]: https://github.com/ros-navigation/navigation2/blob/main/nav2_docking/opennav_docking_bt/src/dock_robot.cpp
[Undock Robot Action]: https://github.com/ros-navigation/navigation2/blob/main/nav2_docking/opennav_docking_bt/src/undock_robot.cpp
[Concatenate Paths Action]: https://github.com/ros-navigation/navigation2/blob/main/nav2_behavior_tree/plugins/action/concatenate_paths_action.cpp
[Get Current Pose Action]: https://github.com/ros-navigation/navigation2/blob/main/nav2_behavior_tree/plugins/action/get_current_pose_action.cpp

<div class="center-table" markdown>

| Condition Plugin Name                                              | Creator                        | Description                                                                                                        |
|--------------------------------------------------------------------|--------------------------------|--------------------------------------------------------------------------------------------------------------------|
| [Goal Reached Condition][]                                         | Carl Delsey                    | Checks if goal is<br>reached within tol.                                                                           |
| [Goal Updated Condition][]                                         | Aitor Miguel Blanco            | Checks if goal is<br>preempted.                                                                                    |
| [Global Updated Goal Condition][]                                  | Joshua Wallace                 | Checks if goal is<br>preempted in the global<br>BT context                                                         |
| [Initial Pose received Condition][]                                | Carl Delsey                    | Checks if initial pose<br>has been set                                                                             |
| [Is Stuck Condition][]                                             | Michael Jeronimo               | Checks if robot is<br>making progress or<br>stuck                                                                  |
| [Transform Available Condition][]                                  | Steve Macenski                 | Checks if a TF<br>transformation is<br>available. When<br>succeeds returns<br>success for subsequent<br>calls.     |
| [Distance Traveled Condition][]                                    | Sarthak Mittal                 | Checks is robot has<br>traveled a given<br>distance.                                                               |
| [Time Expired Condition][]                                         | Sarthak Mittal                 | Checks if a given<br>time period has<br>passed.                                                                    |
| [Is Battery Low Condition][]                                       | Sarthak Mittal                 | Checks if battery<br>percentage is below<br>a specified value.                                                     |
| [Is Path Valid Condition][]                                        | Joshua Wallace                 | Checks if a path is valid by making sure<br>there are no LETHAL obstacles<br>along the path.                       |
| [Path Expiring Timer][]                                            | Joshua Wallace                 | Checks if the timer has<br>expired. The timer is<br>reset if the path gets<br>updated.                             |
| [Are Error Codes Present][]                                        | Joshua Wallace                 | Checks if the specified<br>error codes are<br>present.                                                             |
| [Would A Controller Recovery Help][]                               | Joshua Wallace                 | Checks if a controller<br>recovery could help<br>clear the controller<br>server error code.                        |
| [Would A Planner Recovery Help][]                                  | Joshua Wallace                 | Checks if a planner<br>recovery could help<br>clear the planner<br>server error code.                              |
| [Would A Smoother Recovery Help][]                                 | Joshua Wallace                 | Checks if a Smoother<br>recovery could help<br>clear the smoother<br>server error code.                            |
| [Is Battery Charging Condition][]                                  | Alberto Tudela                 | Checks if the battery<br>is charging.                                                                              |
| [Are Poses Near Condition][]                                       | Steve Macenski                 | Checks if 2 poses are<br>nearby to each other.                                                                     |

</div>

[Goal Reached Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/goal_reached_condition.cpp
[Goal Updated Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/goal_updated_condition.cpp
[Global Updated Goal Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/globally_updated_goal_condition.cpp
[Initial Pose received Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/initial_pose_received_condition.cpp
[Is Stuck Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/is_stuck_condition.cpp
[Transform Available Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/transform_available_condition.cpp
[Distance Traveled Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/distance_traveled_condition.cpp
[Time Expired Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/time_expired_condition.cpp
[Is Battery Low Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/is_battery_low_condition.cpp
[Is Path Valid Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/is_path_valid_condition.cpp
[Path Expiring Timer]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.cpp
[Are Error Codes Present]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/are_error_codes_present_condition.cpp
[Would A Controller Recovery Help]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/would_a_controller_recovery_help_condition.cpp
[Would A Planner Recovery Help]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/would_a_planner_recovery_help_condition.cpp
[Would A Smoother Recovery Help]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/would_a_smoother_recovery_help_condition.cpp
[Is Battery Charging Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/is_battery_charging_condition.cpp
[Are Poses Near Condition]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/condition/are_poses_near_condition.cpp

<div class="center-table" markdown>

| Decorator Plugin Name              | Creator              | Description                                                                                                                      |
|------------------------------------|----------------------|----------------------------------------------------------------------------------------------------------------------------------|
| [Rate Controller][]                | Michael Jeronimo     | Throttles child node to a given<br>rate                                                                                          |
| [Distance Controller][]            | Sarthak Mittal       | Ticks child node based on the<br>distance traveled by the robot                                                                  |
| [Speed Controller][]               | Sarthak Mittal       | Throttles child node to a rate<br>based on current robot speed.                                                                  |
| [Goal Updater][]                   | Francisco Martín     | Updates the goal received via<br>topic subscription.                                                                             |
| [Single Trigger][]                 | Steve Macenski       | Triggers nodes/subtrees below<br>only a single time per BT run.                                                                  |
| [PathLongerOnApproach][]           | Pradheep Padmanabhan | Triggers child nodes if the new<br>global path is significantly<br>larger than the old global path<br>on approach to the goal    |
| [GoalUpdatedController][]          | Sophia Koffler       | Ticks child node if the goal<br>has been updated                                                                                 |

</div>

[Rate Controller]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/rate_controller.cpp
[Distance Controller]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/distance_controller.cpp
[Speed Controller]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/speed_controller.cpp
[Goal Updater]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/goal_updater_node.cpp
[Single Trigger]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/single_trigger_node.cpp
[PathLongerOnApproach]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/path_longer_on_approach.cpp
[GoalUpdatedController]: https://github.com/ros-navigation/navigation2/blob/main/nav2_behavior_tree/plugins/decorator/goal_updated_controller.cpp

<div class="center-table" markdown>

| Control Plugin Name               | Creator                | Description                                                                                                                                                                                                                                                     |
|-----------------------------------|------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Pipeline Sequence][]             | Carl Delsey            | A variant of a sequence node that<br>will re-tick previous children<br>even if another child is running                                                                                                                                                         |
| [Recovery][]                      | Carl Delsey            | Node must contain 2 children<br>and returns success if first<br>succeeds. If first fails, the<br>second will be ticked. If<br>successful, it will retry the<br>first and then return its value                                                                  |
| [Round Robin][]                   | Mohammad Haghighipanah | Will tick `i` th child until<br>a result and move on to `i+1`                                                                                                                                                                                                   |

</div>

[Pipeline Sequence]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/control/pipeline_sequence.cpp
[Recovery]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/control/recovery_node.cpp
[Round Robin]: https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins/control/round_robin_node.cpp

## Route Plugins

### Edge Scorers

<div class="center-table" markdown>

| Plugin Name                | Creator        | Description                                                                                     |
|----------------------------|----------------|-------------------------------------------------------------------------------------------------|
| DistanceScorer             | Steve Macenski | Scores an edge's length,<br>optionally scaled by relative<br>speed limits.                      |
| TimeScorer                 | Steve Macenski | Scores and edge traversal time<br>using absolute speed limits or<br>previous traversal times.   |
| PenaltyScorer              | Steve Macenski | Scores using a static semantic<br>penalty.                                                      |
| SemanticScorer             | Steve Macenski | Scores using stored semantic data<br>regarding the edge and/or nodes.                           |
| StartPoseOrientationScorer | Alex Yuen      | Scores based on the initial pose<br>and start edge orientations.                                |
| GoalPoseOrientationScorer  | Alex Yuen      | Scores based on the goal pose and<br>goal edge orientations.                                    |
| DynamicEdgesScorer         | Steve Macenski | Scores based on a dynamically set<br>service cost and/or closure.                               |

</div>

### Route Operations

<div class="center-table" markdown>

| Plugin Name      | Creator        | Description                                                                  |
|------------------|----------------|------------------------------------------------------------------------------|
| AdjustSpeedLimit | Steve Macenski | Adjusts robot speed limits using<br>an edge's semantic data.                 |
| CollisionMoniter | Steve Macenski | Checks for collision in the<br>immediate future which tracking<br>a route.   |
| TimeMarker       | Steve Macenski | Records the traversal time for an<br>edge in the edge's metadata.            |
| ReroutingService | Steve Macenski | Triggers a rereoute from an<br>external server.                              |
| TriggerEvent     | Steve Macenski | Triggers an event based on a<br>configurable server name.                    |

</div>

### Graph File Parsers

Currently, only `geojson` parsing is supported.
