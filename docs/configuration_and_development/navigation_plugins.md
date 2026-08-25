# Navigation Plugins { #navigation-plugins }

There are a number of plugin interfaces for users to create their own custom applications or algorithms with.
Namely, the costmap layer, planner, controller, behavior tree, and behavior plugins.
A list of all known plugins are listed here below for ROS 2 Navigation.
If you know of a plugin, or you have created a new plugin, please consider submitting a pull request with that information.

This file can be found and edited under `docs/configuration_and_development/navigation_plugins/index.md`.
For tutorials on creating your own plugins, please see:

- [Writing a New Costmap2D Plugin][writing-a-new-costmap-2d-plugin]
- [Writing a New Behavior Tree Plugin][writing-a-new-behavior-tree-plugin]
- [Writing a New Controller Plugin][writing-a-new-controller-plugin]
- [Writing a New Planner Plugin][writing-a-new-planner-plugin]
- [Writing a New Behavior Plugin][writing-a-new-behavior-plugin]
- [Writing a New Navigator Plugin][writing-a-new-navigator-plugin]

## Behavior-Tree Navigators

<div class="center-table" markdown>

| Plugin Name                       | Creator        | Description                                                           |
|-----------------------------------|----------------|-----------------------------------------------------------------------|
| [NavigateToPoseNavigator][]       | Steve Macenski | Point-to-point navigation via a behavior tree action server           |
| [NavigateThroughPosesNavigator][] | Steve Macenski | Point-through-points navigation via a behavior tree action server     |
| [CoverageNavigator][]             | Steve Macenski | Complete coverage navigation (Cartesian or GPS) via a BTs             |

</div>

[NavigateToPoseNavigator]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_bt_navigator/src/navigators
[NavigateThroughPosesNavigator]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_bt_navigator/src/navigators
[CoverageNavigator]: https://github.com/open-navigation/opennav_coverage/tree/lyrical/opennav_coverage_navigator

## Costmap Layers

<div class="center-table" markdown>

| Plugin Name                            | Creator                    | Description                                                                                                                                                                |
|----------------------------------------|----------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Voxel Layer][]                        | Eitan Marder-Eppstein      | Maintains persistent 3D voxel layer using depth and laser sensor readings and raycasting to clear free space                                                               |
| [Range Layer][]                        | David Lu                   | Uses a probabilistic model to put data from sensors that publish range msgs on the costmap                                                                                 |
| [Static Layer][]                       | Eitan Marder-Eppstein      | Gets static `map` and loads occupancy information into costmap                                                                                                             |
| [Inflation Layer][]                    | Tony Najjar                | Inflates lethal obstacles in costmap with exponential decay (with the option to use OpenMP for parallelization)                                                            |
| [Legacy Inflation Layer][]             | Eitan Marder-Eppstein      | Inflates lethal obstacles in costmap with exponential decay                                                                                                                |
| [Asymmetric Inflation Layer][]         | Marc Blöchlinger           | Uses the global plan to asymmetrically inflate lethal obstacles depending on path side                                                                                     |
| [Obstacle Layer][]                     | Eitan Marder-Eppstein      | Maintains persistent 2D costmap from 2D laser scans with raycasting to clear free space                                                                                    |
| [Spatio-Temporal Voxel Layer][]        | Steve Macenski             | Maintains temporal 3D sparse volumetric voxel grid with decay through sensor models                                                                                        |
| [Non-Persistent Voxel Layer][]         | Steve Macenski             | Maintains 3D occupancy grid consisting only of the most sets of measurements                                                                                               |
| [Denoise Layer][]                      | Andrey Ryzhikov            | Filters noise-induced standalone obstacles or small obstacles groups                                                                                                       |
| [Plugin Container Layer][]             | Alexander Yuen             | Combines the different costmap layers specified under this layer in order populate the same costmap with different isolated combinations of costmap layers                 |
| [Ground Consistency Layer][]           | Muhammad Haider Khan Lodhi | Height-aware costmap layer using 3D ground segmentation. Pair with Inflation Layer for terrain-aware navigation.                                                           |
| [Semantic Segmentation Layer][]        | Pedro Gonzalez             | Vision-based semantic segmentation costmap layer using per-pixel class masks and registered pointclouds for terrain-aware navigation.                                      |
| [Virtual Layer][]                      | Sherif Fathey              | Creates dynamic virtual cost zones and restriction areas using polygons, lines, and circles                                                                                |

</div>

[Voxel Layer]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/voxel_layer.cpp
[Range Layer]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/range_sensor_layer.cpp
[Static Layer]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/static_layer.cpp
[Inflation Layer]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/inflation_layer.cpp
[Legacy Inflation Layer]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/legacy_inflation_layer.cpp
[Asymmetric Inflation Layer]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/asymmetric_inflation_layer.cpp
[Obstacle Layer]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/obstacle_layer.cpp
[Spatio-Temporal Voxel Layer]: https://github.com/SteveMacenski/spatio_temporal_voxel_layer/
[Non-Persistent Voxel Layer]: https://github.com/SteveMacenski/nonpersistent_voxel_layer
[Denoise Layer]: https://github.com/ryzhikovas/navigation2/tree/feature-costmap2d-denoise/nav2_costmap_2d/plugins/denoise_layer.cpp
[Plugin Container Layer]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/plugin_container_layer.cpp
[Ground Consistency Layer]: https://github.com/dfki-ric/nav2_ground_consistency_costmap_plugin
[Semantic Segmentation Layer]: https://github.com/kiwicampus/semantic_segmentation_layer
[Virtual Layer]: https://github.com/SherifFathey/nav2-virtual-layer

## Costmap Filters { #navigation-plugins-costmap-filters }

<div class="center-table" markdown>

| Plugin Name               | Creator           | Description                                                                            |
|---------------------------|-------------------|----------------------------------------------------------------------------------------|
| [Keepout Filter][]        | Alexey Merzlyakov | Maintains keep-out/safety zones and preferred lanes for moving                         |
| [Speed Filter][]          | Alexey Merzlyakov | Limits maximum velocity of robot in speed restriction areas                            |
| [Binary Filter][]         | Alexey Merzlyakov | Enables binary (boolean) mask behavior to trigger actions                              |
| [Zone Parameter Filter][] | Akihiko Komada    | Applies a configured set of ROS parameters based on the mask value at the robot's pose |

</div>

[Keepout Filter]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp
[Speed Filter]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/costmap_filters/speed_filter.cpp
[Binary Filter]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/costmap_filters/binary_filter.cpp
[Zone Parameter Filter]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_costmap_2d/plugins/costmap_filters/zone_parameter_filter.cpp

## Controllers

<div class="center-table" markdown>

| Plugin Name                               | Creator                             | Description                                                                                            | Drivetrain support                                           |
|-------------------------------------------|-------------------------------------|--------------------------------------------------------------------------------------------------------|--------------------------------------------------------------|
| [DWB Controller][]                        | David Lu!!                          | A highly configurable  DWA implementation with plugin interfaces                                       | Differential, Omnidirectional, Legged                        |
| [TEB Controller][]                        | Christoph Rösmann                   | A MPC-like controller suitable for ackermann, differential, and holonomic robots.                      | **Ackermann**, Legged, Omnidirectional, Differential         |
| [Regulated Pure Pursuit][]                | Steve Macenski                      | A service / industrial robot variation on the pure pursuit algorithm with adaptive features.           | **Ackermann**, Legged, Differential                          |
| [MPPI Controller][]                       | Steve Macenski and Aleksei Budyakov | A predictive MPC controller with modular & custom cost functions that can accomplish many tasks.       | Differential, Omni, **Ackermann**                            |
| [Rotation Shim Controller][]              | Steve Macenski                      | A "shim" controller to rotate to path heading before passing to main controller for  tracking.         | Differential, Omni, model rotate in place                    |
| [Graceful Controller][]                   | Alberto Tudela                      | A controller based on a pose-following control law to generate smooth trajectories.                    | Differential, Omni, Legged                                   |
| [Vector Pursuit Controller][]             | Black Coffee Robotics               | A controller based on the vector pursuit algorithm useful for high speed accurate path tracking.       | Differential, Ackermann, Legged                              |

</div>

[DWB Controller]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_dwb_controller
[TEB Controller]: https://github.com/rst-tu-dortmund/teb_local_planner
[Regulated Pure Pursuit]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_regulated_pure_pursuit_controller
[MPPI Controller]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_mppi_controller
[Rotation Shim Controller]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_rotation_shim_controller
[Graceful Controller]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_graceful_controller
[Vector Pursuit Controller]: https://github.com/blackcoffeerobotics/vector_pursuit_controller

## Planners

<div class="center-table" markdown>

| Plugin Name                                         | Creator                               | Description                                                                                                                                                                                                                                                          | Drivetrain support                                                                   |
|-----------------------------------------------------|---------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------|
| [NavFn Planner][]                                   | Eitan Marder-Eppstein & Kurt Konolige | A navigation function using A\* or Dijkstras expansion, assumes 2D holonomic particle                                                                                                                                                                                | Differential, Omnidirectional, Legged                                                |
| [SmacPlannerHybrid][] <br> (formerly *SmacPlanner*) | Steve Macenski                        | A SE2 Hybrid-A\* implementation using either Dubin or Reeds-shepp motion models with smoother and multi-resolution query. Cars, car-like, and ackermann vehicles. Kinematically feasible.                                                                            | **Ackermann**, Differential, Omnidirectional, Legged                                 |
| [SmacPlanner2D][]                                   | Steve Macenski                        | A 2D A\* implementation Using either 4 or 8 connected neighborhoods with smoother and multi-resolution query                                                                                                                                                         | Differential, Omnidirectional, Legged                                                |
| [SmacPlannerLattice][]                              | Steve Macenski                        | An implementation of State Lattice Planner using pre-generated minimum control sets for kinematically feasible planning with any type of vehicle imaginable. Includes generator script for Ackermann, diff, omni, and legged robots.                                 | Differential, Omnidirectional, Ackermann, Legged, Arbitrary / Custom                 |
| [ThetaStarPlanner][]                                | Anshumaan Singh                       | An implementation of Theta\* using either 4 or 8 connected neighborhoods, assumes the robot as a 2D holonomic particle                                                                                                                                               | Differential, Omnidirectional                                                        |

</div>

[NavFn Planner]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_navfn_planner
[SmacPlannerHybrid]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_smac_planner
[SmacPlanner2D]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_smac_planner
[SmacPlannerLattice]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_smac_planner
[ThetaStarPlanner]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_theta_star_planner

## Smoothers

<div class="center-table" markdown>

| Plugin Name                          | Creator                          | Description                                                                                                                                                                               |
|--------------------------------------|----------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Simple Smoother][]                  | Steve Macenski                   | A simple path smoother for infeasible (e.g. 2D) planners                                                                                                                                  |
| [Constrained Smoother][]             | Matej Vargovcik & Steve Macenski | A path smoother using a constraints problem solver to optimize various criteria such as smoothness or distance from obstacles, maintaining minimum turning radius                         |
| [Savitzky-Golay Smoother][]          | Steve Macenski                   | A path smoother using a Savitzky-Golay filter to smooth the path via digital signal processing to remove noise from the path.                                                             |

</div>

[Simple Smoother]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_smoother
[Constrained Smoother]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_constrained_smoother
[Savitzky-Golay Smoother]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_smoother

## Behaviors

<div class="center-table" markdown>

| Plugin Name                       | Creator               | Description                                                                                                                            |
|-----------------------------------|-----------------------|----------------------------------------------------------------------------------------------------------------------------------------|
| [Clear Costmap][]                 | Eitan Marder-Eppstein | A service to clear the given costmap in case of incorrect perception or robot is stuck                                                 |
| [Spin][]                          | Steve Macenski        | Rotate behavior of configurable angles to clear out free space and nudge robot out of potential local failures                         |
| [Back Up][]                       | Brian Wilcox          | Back up behavior of configurable distance to back out of a situation where the robot is stuck                                          |
| [Wait][]                          | Steve Macenski        | Wait behavior with configurable time to wait in case of time based obstacle like human traffic or getting more sensor data             |
| [Drive On Heading][]              | Joshua Wallace        | Drive on heading behavior with configurable distance to drive                                                                          |
| [Assisted Teleop][]               | Joshua Wallace        | AssistedTeleop behavior that scales teleop commands to prevent collisions.                                                             |

</div>

[Clear Costmap]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_costmap_2d/src/clear_costmap_service.cpp
[Spin]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behaviors/plugins
[Back Up]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behaviors/plugins
[Wait]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behaviors/plugins
[Drive On Heading]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behaviors/plugins
[Assisted Teleop]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behaviors/plugins

## Waypoint Task Executors

<div class="center-table" markdown>

| Plugin Name          | Creator        | Description                                                                           |
|----------------------|----------------|---------------------------------------------------------------------------------------|
| [WaitAtWaypoint][]   | Fetullah Atas  | A plugin to execute a wait behavior  on waypoint arrivals.                            |
| [PhotoAtWaypoint][]  | Fetullah Atas  | A plugin to take and save photos to specified directory on waypoint arrivals.         |
| [InputAtWaypoint][]  | Steve Macenski | A plugin to wait for user input before moving onto the next waypoint.                 |

</div>

[WaitAtWaypoint]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_waypoint_follower/plugins/wait_at_waypoint.cpp
[PhotoAtWaypoint]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_waypoint_follower/plugins/photo_at_waypoint.cpp
[InputAtWaypoint]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_waypoint_follower/plugins/input_at_waypoint.cpp

## Goal Checkers

<div class="center-table" markdown>

| Plugin Name                        | Creator                       | Description                                                                                                                          |
|------------------------------------|-------------------------------|--------------------------------------------------------------------------------------------------------------------------------------|
| [SimpleGoalChecker][]              | David Lu!!                    | A plugin check whether robot is within translational distance and rotational distance of goal.                                       |
| [StoppedGoalChecker][]             | David Lu!!                    | A plugin check whether robot is within translational distance, rotational distance of goal, and velocity threshold.                  |
| [PositionGoalChecker][]            | Prabhav Saxena                | A plugin check whether robot is within translational distance of goal, without requiring rotational convergence.                     |
| [AxisGoalChecker][]                | Guillaume Doisy & Tony Najjar | A plugin check whether robot is within tolerance along the path direction and perpendicular to it (cross-track).                     |
| [AdaptiveToleranceGoalChecker][]   | David Grbac                   | A plugin check whether robot is within translational distance (using two tolerance levels) and rotational distance of goal.          |

</div>

[SimpleGoalChecker]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_controller/plugins/simple_goal_checker.cpp
[StoppedGoalChecker]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_controller/plugins/stopped_goal_checker.cpp
[PositionGoalChecker]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_controller/plugins/position_goal_checker.cpp
[AxisGoalChecker]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_controller/plugins/axis_goal_checker.cpp
[AdaptiveToleranceGoalChecker]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_controller/plugins/adaptive_tolerance_goal_checker.cpp

## Progress Checkers

<div class="center-table" markdown>

| Plugin Name                | Creator         | Description                                                                                                                                  |
|----------------------------|-----------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| [SimpleProgressChecker][]  | David Lu!!      | A plugin to check whether the robot was able to move a minimum distance in a given time to make progress towards a goal                      |
| [PoseProgressChecker][]    | Guillaume Doisy | A plugin to check whether the robot was able to move a minimum distance or angle in a given time to make progress towards a goal             |

</div>

[SimpleProgressChecker]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_controller/plugins/simple_progress_checker.cpp
[PoseProgressChecker]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_controller/plugins/pose_progress_checker.cpp

## Path Handlers

<div class="center-table" markdown>

| Plugin Name             | Creator                        | Description                                                                                                                                                                                       |
|-------------------------|--------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [FeasiblePathHandler][] | Maurice Alexander Purnawan     | A plugin that transforms global plan to the local costmap frame, prunes it to the relevant portion within the costmap bounds, and handles in-place rotation and cusp pruning.                     |

</div>

[FeasiblePathHandler]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_controller/plugins/feasible_path_handler.cpp

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
| [Navigate to Pose Action][]                                            | Michael Jeronimo               | BT Node for other BehaviorTree.CPP BTs to call Nav2 as a subtree action                                             |
| [Reinitialize Global Localization Service][]                           | Carl Delsey                    | Reinitialize AMCL to a new pose                                                                                            |
| [Spin Action][]                                                        | Carl Delsey                    | Calls spin behavior action                                                                                                 |
| [Wait Action][]                                                        | Steve Macenski                 | Calls wait behavior action                                                                                                 |
| [Truncate Path][]                                                      | Francisco Martín               | Modifies a path making it shorter                                                                                          |
| [Truncate Path Local][]                                                | Matej Vargovcik                | Extracts a path section around robot                                                                                       |
| [Planner Selector][]                                                   | Pablo Iñigo Blasco             | Selects the global planner based on a topic input, otherwises uses a default planner id                                    |
| [Controller Selector][]                                                | Pablo Iñigo Blasco             | Selects the controller based on a topic input, otherwises uses a default controller id                                     |
| [Goal Checker Selector][]                                              | Pablo Iñigo Blasco             | Selects the goal checker based on a topic input, otherwises uses a default goal checker id                                 |
| [Smoother Selector][]                                                  | Owen Hooper                    | Selects the smoother based on a topic input, otherwises uses a default smoother id                                         |
| [Progress Checker Selector][]                                          | Steve Macenski                 | Selects the progress checker based on a topic input, otherwises uses a default progress checker id                         |
| [Path Handler Selector][]                                              | Maurice Alexander Purnawan     | Selects the path handler based on a topic input, otherwises uses a default path handler id                                 |
| [Navigate Through Poses][]                                             | Steve Macenski                 | BT Node for other BehaviorTree.CPP BTs to call Nav2's NavThroughPoses action                                               |
| [Remove Passed Goals][]                                                | Steve Macenski                 | Removes goal poses passed or within a tolerance for culling old viapoints from path re-planning                            |
| [Remove In Collision Goals][]                                          | Tony Najjar                    | Removes goal poses that have a footprint or point cost above a threshold.                                                  |
| [Compute Path Through Poses][]                                         | Steve Macenski                 | Computes a path through a set of poses rather than a single end goal pose using the planner plugin specified               |
| [Compute Route][]                                                      | Steve Macenski                 | Computes a Route through a navigation graph and returns both a dense path and set of sparse route nodes and edges.         |
| [Compute And Track Route][]                                            | Steve Macenski                 | Computes a Route as above, but also actively tracks progress and triggers route contextual semantic operations.            |
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
| [Append Goal Pose To Goals Action][]                                   | Steve Macenski                 | Appends a goal pose to a goals vector                                                                                      |
| [Extract Route Nodes To Goals Action][]                                | Steve Macenski                 | Converts Route Nodes to Goals                                                                                              |
| [Get Next Few Goals Action][]                                          | Steve Macenski                 | Obtains the next N goals in a goal vector                                                                                  |
| [Toggle Collision Monitor Service][]                                   | David Grbac                    | Calls toggle collision monitor service                                                                                     |
| [Follow Object][]                                                      | Alberto Tudela                 | Dynamically follows an object                                                                                              |
| [Cancel Follow Object][]                                               | Alberto Tudela                 | Cancels follow object action                                                                                               |
| [Validate Path][]                                                      | Joshua Wallace                 | Checks if a path is valid by making sure there are no LETHAL obstacles along the path.                                     |
| [Check Stop Status][]                                                  | Tony Najjar                    | Checks if robot is stopped for a duration                                                                                  |
| [Check Pose Occupancy][]                                               | Maurice Alexander Purnawan     | Checks if a pose is occupied.                                                                                              |

</div>

[Back Up Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/back_up_action.cpp
[Drive On Heading Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/drive_on_heading_action.cpp
[Assisted Teleop Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/assisted_teleop_action.cpp
[Clear Entire Costmap Service]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
[Clear Costmap Except Region Service]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
[Clear Costmap Around Robot Service]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
[Compute Path to Pose Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp
[Smooth Path Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/smooth_path_action.cpp
[Follow Path Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/follow_path_action.cpp
[Navigate to Pose Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/navigate_to_pose_action.cpp
[Reinitialize Global Localization Service]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/reinitialize_global_localization_service.cpp
[Spin Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/spin_action.cpp
[Wait Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/wait_action.cpp
[Truncate Path]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/truncate_path_action.cpp
[Truncate Path Local]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/truncate_path_local_action.cpp
[Planner Selector]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/planner_selector_node.cpp
[Controller Selector]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/controller_selector_node.cpp
[Goal Checker Selector]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/goal_checker_selector_node.cpp
[Smoother Selector]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/smoother_selector_node.cpp
[Progress Checker Selector]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/progress_checker_selector_node.cpp
[Path Handler Selector]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/path_handler_selector_node.cpp
[Navigate Through Poses]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/navigate_through_poses_action.cpp
[Remove Passed Goals]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/remove_passed_goals_action.cpp
[Remove In Collision Goals]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/remove_in_collision_goals_action.cpp
[Compute Path Through Poses]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/compute_path_through_poses_action.cpp
[Compute Route]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/compute_route_action.cpp
[Compute And Track Route]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/compute_and_track_route_action.cpp
[Cancel Control Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/controller_cancel_node.cpp
[Cancel BackUp Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/back_up_cancel_node.cpp
[Cancel Spin Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/spin_cancel_node.cpp
[Cancel Wait Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/wait_cancel_node.cpp
[Cancel Route Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/compute_and_track_route_cancel_node.cpp
[Cancel Drive on Heading Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/drive_on_heading_cancel_node.cpp
[Cancel Assisted Teleop Action]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/assisted_teleop_cancel_node.cpp
[Cancel Complete Coverage Action]: https://github.com/open-navigation/opennav_coverage/blob/lyrical/opennav_coverage_bt/src/cancel_complete_coverage_path.cpp
[Compute Complete Coverage Path Action]: https://github.com/open-navigation/opennav_coverage/blob/lyrical/opennav_coverage_bt/src/compute_complete_coverage_path.cpp
[Get Pose From Path Action]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_behavior_tree/plugins/action/get_pose_from_path_action.cpp
[Dock Robot Action]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_docking/opennav_docking_bt/src/dock_robot.cpp
[Undock Robot Action]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_docking/opennav_docking_bt/src/undock_robot.cpp
[Concatenate Paths Action]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_behavior_tree/plugins/action/concatenate_paths_action.cpp
[Get Current Pose Action]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_behavior_tree/plugins/action/get_current_pose_action.cpp
[Append Goal Pose To Goals Action]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_behavior_tree/plugins/action/append_goal_pose_to_goals_action.cpp
[Extract Route Nodes To Goals Action]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_behavior_tree/plugins/action/extract_route_nodes_as_goals_action.cpp
[Get Next Few Goals Action]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_behavior_tree/plugins/action/get_next_few_goals_action.cpp
[Toggle Collision Monitor Service]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_behavior_tree/plugins/action/toggle_collision_monitor_service.cpp
[Follow Object]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_behavior_tree/plugins/action/follow_object_action.cpp
[Cancel Follow Object]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_behavior_tree/plugins/action/follow_object_cancel_node.cpp
[Validate Path]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/validate_path_action.cpp
[Check Stop Status]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/check_stop_status_action.cpp
[Check Pose Occupancy]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/action/check_pose_occupancy_action.cpp

<div class="center-table" markdown>

| Condition Plugin Name                                              | Creator                        | Description                                                                                                        |
|--------------------------------------------------------------------|--------------------------------|--------------------------------------------------------------------------------------------------------------------|
| [Goal Reached Condition][]                                         | Carl Delsey                    | Checks if goal is reached within tol.                                                                              |
| [Goal Updated Condition][]                                         | Aitor Miguel Blanco            | Checks if goal is preempted.                                                                                       |
| [Global Updated Goal Condition][]                                  | Joshua Wallace                 | Checks if goal is preempted in the global BT context                                                               |
| [Initial Pose received Condition][]                                | Carl Delsey                    | Checks if initial pose has been set                                                                                |
| [Is Stuck Condition][]                                             | Michael Jeronimo               | Checks if robot is making progress or stuck                                                                        |
| [Transform Available Condition][]                                  | Steve Macenski                 | Checks if a TF transformation is available. When succeeds returns success for subsequent calls.                    |
| [Distance Traveled Condition][]                                    | Sarthak Mittal                 | Checks is robot has traveled a given distance.                                                                     |
| [Time Expired Condition][]                                         | Sarthak Mittal                 | Checks if a given time period has passed.                                                                          |
| [Is Battery Low Condition][]                                       | Sarthak Mittal                 | Checks if battery percentage is below a specified value.                                                           |
| [Path Expiring Timer][]                                            | Joshua Wallace                 | Checks if the timer has expired. The timer is reset if the path gets updated.                                      |
| [Are Error Codes Present][]                                        | Joshua Wallace                 | Checks if the specified error codes are present.                                                                   |
| [Would A Controller Recovery Help][]                               | Joshua Wallace                 | Checks if a controller recovery could help clear the controller server error code.                                 |
| [Would A Planner Recovery Help][]                                  | Joshua Wallace                 | Checks if a planner recovery could help clear the planner server error code.                                       |
| [Would A Smoother Recovery Help][]                                 | Joshua Wallace                 | Checks if a Smoother recovery could help clear the smoother server error code.                                     |
| [Would A Route Recovery Help][]                                    | Steve Macenski                 | Checks if a Route recovery could help clear the route server error code.                                           |
| [Is Battery Charging Condition][]                                  | Alberto Tudela                 | Checks if the battery is charging.                                                                                 |
| [Are Poses Near Condition][]                                       | Steve Macenski                 | Checks if 2 poses are nearby to each other.                                                                        |
| [Is Goal Nearby Condition][]                                       | Jakub Chudziński               | Checks if the robot is near the goal based on remaining path length.                                               |
| [Is Within Path Tracking Bounds Condition][]                       | Berkan Tali                    | Checks if the robot is within bounds for path tracking.                                                            |

</div>

[Goal Reached Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/goal_reached_condition.cpp
[Goal Updated Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/goal_updated_condition.cpp
[Global Updated Goal Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/globally_updated_goal_condition.cpp
[Initial Pose received Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/initial_pose_received_condition.cpp
[Is Stuck Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/is_stuck_condition.cpp
[Transform Available Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/transform_available_condition.cpp
[Distance Traveled Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/distance_traveled_condition.cpp
[Time Expired Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/time_expired_condition.cpp
[Is Battery Low Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/is_battery_low_condition.cpp
[Path Expiring Timer]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.cpp
[Are Error Codes Present]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/are_error_codes_present_condition.cpp
[Would A Controller Recovery Help]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/would_a_controller_recovery_help_condition.cpp
[Would A Planner Recovery Help]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/would_a_planner_recovery_help_condition.cpp
[Would A Smoother Recovery Help]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/would_a_smoother_recovery_help_condition.cpp
[Would A Route Recovery Help]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/would_a_route_recovery_help_condition.cpp
[Is Battery Charging Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/is_battery_charging_condition.cpp
[Are Poses Near Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/are_poses_near_condition.cpp
[Is Goal Nearby Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/is_goal_nearby_condition.cpp
[Is Within Path Tracking Bounds Condition]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/condition/is_within_path_tracking_bounds_condition.cpp

<div class="center-table" markdown>

| Decorator Plugin Name              | Creator              | Description                                                                                                                      |
|------------------------------------|----------------------|----------------------------------------------------------------------------------------------------------------------------------|
| [Rate Controller][]                | Michael Jeronimo     | Throttles child node to a given rate                                                                                             |
| [Distance Controller][]            | Sarthak Mittal       | Ticks child node based on the distance traveled by the robot                                                                     |
| [Speed Controller][]               | Sarthak Mittal       | Throttles child node to a rate based on current robot speed.                                                                     |
| [Goal Updater][]                   | Francisco Martín     | Updates the goal received via topic subscription.                                                                                |
| [Single Trigger][]                 | Steve Macenski       | Triggers nodes/subtrees below only a single time per BT run.                                                                     |
| [PathLongerOnApproach][]           | Pradheep Padmanabhan | Triggers child nodes if the new global path is significantly larger than the old global path on approach to the goal             |
| [GoalUpdatedController][]          | Sophia Koffler       | Ticks child node if the goal has been updated                                                                                    |

</div>

[Rate Controller]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/decorator/rate_controller.cpp
[Distance Controller]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/decorator/distance_controller.cpp
[Speed Controller]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/decorator/speed_controller.cpp
[Goal Updater]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/decorator/goal_updater_node.cpp
[Single Trigger]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/decorator/single_trigger_node.cpp
[PathLongerOnApproach]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/decorator/path_longer_on_approach.cpp
[GoalUpdatedController]: https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_behavior_tree/plugins/decorator/goal_updated_controller.cpp

<div class="center-table" markdown>

| Control Plugin Name               | Creator                | Description                                                                                                                                                                                                                                                     |
|-----------------------------------|------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Pipeline Sequence][]             | Carl Delsey            | A variant of a sequence node that will re-tick previous children even if another child is running                                                                                                                                                               |
| [Recovery][]                      | Carl Delsey            | Node must contain 2 children and returns success if first succeeds. If first fails, the second will be ticked. If successful, it will retry the first and then return its value                                                                                 |
| [Round Robin][]                   | Mohammad Haghighipanah | Will tick `i` th child until a result and move on to `i+1`                                                                                                                                                                                                      |
| [Nonblocking Sequence][]          | Alexander Yuen         | A variant of a sequence node that will tick through the whole sequence even if a child returns running. On reticks of this control node, successful children will be ticked once again to prevent a stale state from being latched.                             |
| [Persistent Sequence][]           | Enjoy Robotics         | A variant of a sequence node that exposes `current_child_idx` as a bidirectional port.                                                                                                                                                                          |
| [Pause Resume Controller][]       | Enjoy Robotics         | Controlled through service calls to pause and resume the execution of the tree.                                                                                                                                                                                 |

</div>

[Pipeline Sequence]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/control/pipeline_sequence.cpp
[Recovery]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/control/recovery_node.cpp
[Round Robin]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/control/round_robin_node.cpp
[Nonblocking Sequence]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/control/nonblocking_sequence.cpp
[Persistent Sequence]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/control/persistent_sequence.cpp
[Pause Resume Controller]: https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_behavior_tree/plugins/control/pause_resume_controller.cpp

## Route Plugins

### Edge Scorers

<div class="center-table" markdown>

| Plugin Name                | Creator        | Description                                                                                     |
|----------------------------|----------------|-------------------------------------------------------------------------------------------------|
| DistanceScorer             | Steve Macenski | Scores an edge's length, optionally scaled by relative speed limits.                            |
| TimeScorer                 | Steve Macenski | Scores and edge traversal time using absolute speed limits or previous traversal times.         |
| PenaltyScorer              | Steve Macenski | Scores using a static semantic penalty.                                                         |
| SemanticScorer             | Steve Macenski | Scores using stored semantic data regarding the edge and/or nodes.                              |
| StartPoseOrientationScorer | Alex Yuen      | Scores based on the initial pose and start edge orientations.                                   |
| GoalPoseOrientationScorer  | Alex Yuen      | Scores based on the goal pose and goal edge orientations.                                       |
| DynamicEdgesScorer         | Steve Macenski | Scores based on a dynamically set service cost and/or closure.                                  |

</div>

### Route Operations

<div class="center-table" markdown>

| Plugin Name      | Creator        | Description                                                                  |
|------------------|----------------|------------------------------------------------------------------------------|
| AdjustSpeedLimit | Steve Macenski | Adjusts robot speed limits using an edge's semantic data.                    |
| CollisionMoniter | Steve Macenski | Checks for collision in the immediate future which tracking a route.         |
| TimeMarker       | Steve Macenski | Records the traversal time for an edge in the edge's metadata.               |
| ReroutingService | Steve Macenski | Triggers a rereoute from an external server.                                 |
| TriggerEvent     | Steve Macenski | Triggers an event based on a configurable server name.                       |

</div>

### Graph File Parsers

Currently, only `geojson` parsing is supported.
