.. _plugins:

Navigation Plugins
##################

There are a number of plugin interfaces for users to create their own custom applications or algorithms with.
Namely, the costmap layer, planner, controller, behavior tree, and behavior plugins.
A list of all known plugins are listed here below for ROS 2 Navigation.
If you know of a plugin, or you have created a new plugin, please consider submitting a pull request with that information.

This file can be found and editted under ``sphinx_docs/plugins/index.rst``.
For tutorials on creating your own plugins, please see :ref:`writing_new_costmap2d_plugin`, :ref:`writing_new_nbt_plugin`, :ref:`writing_new_nav2controller_plugin`, :ref:`writing_new_nav2planner_plugin`, :ref:`writing_new_behavior_plugin`, or :ref:`writing_new_nav2navigator_plugin`.

Behavior-Tree Navigators
========================

+----------------------------------+--------------------+-----------------------------------+
|    Plugin Name                   |      Creator       |       Description                 |
+==================================+====================+===================================+
| `NavigateToPoseNavigator`_       | Steve Macenski     | Point-to-point navigation via a   |
|                                  |                    | behavior tree action server       |
+----------------------------------+--------------------+-----------------------------------+
| `NavigateThroughPosesNavigator`_ | Steve Macenski     | Point-through-points navigation   |
|                                  |                    | via a behavior tree action server |
+----------------------------------+--------------------+-----------------------------------+

.. _NavigateToPoseNavigator: https://github.com/ros-planning/navigation2/tree/main/nav2_bt_navigator/src/navigators
.. _NavigateThroughPosesNavigator: https://github.com/ros-planning/navigation2/tree/main/nav2_bt_navigator/src/navigators


Costmap Layers
==============

+--------------------------------+------------------------+----------------------------------+
|            Plugin Name         |         Creator        |       Description                |
+================================+========================+==================================+
| `Voxel Layer`_                 | Eitan Marder-Eppstein  | Maintains persistant             |
|                                |                        | 3D voxel layer using depth and   |
|                                |                        | laser sensor readings and        |
|                                |                        | raycasting to clear free space   |
+--------------------------------+------------------------+----------------------------------+
| `Range Layer`_                 | David Lu               | Uses a probabalistic model to    |
|                                |                        | put data from sensors that       |
|                                |                        | publish range msgs on the costmap|
+--------------------------------+------------------------+----------------------------------+
| `Static Layer`_                | Eitan Marder-Eppstein  | Gets static ``map`` and loads    |
|                                |                        | occupancy information into       |
|                                |                        | costmap                          |
+--------------------------------+------------------------+----------------------------------+
| `Inflation Layer`_             | Eitan Marder-Eppstein  | Inflates lethal obstacles in     |
|                                |                        | costmap with exponential decay   |
+--------------------------------+------------------------+----------------------------------+
|  `Obstacle Layer`_             | Eitan Marder-Eppstein  | Maintains persistent 2D costmap  |
|                                |                        | from 2D laser scans with         |
|                                |                        | raycasting to clear free space   |
+--------------------------------+------------------------+----------------------------------+
| `Spatio-Temporal Voxel Layer`_ |  Steve Macenski        | Maintains temporal 3D sparse     |
|                                |                        | volumetric voxel grid with decay |
|                                |                        | through sensor models            |
+--------------------------------+------------------------+----------------------------------+
| `Non-Persistent Voxel Layer`_  |  Steve Macenski        | Maintains 3D occupancy grid      |
|                                |                        | consisting only of the most      |
|                                |                        | sets of measurements             |
+--------------------------------+------------------------+----------------------------------+
| `Denoise Layer`_               |  Andrey Ryzhikov       | Filters noise-induced            |
|                                |                        | standalone obstacles or small    |
|                                |                        | obstacles groups                 |
+--------------------------------+------------------------+----------------------------------+

.. _Voxel Layer: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/voxel_layer.cpp
.. _Static Layer: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/static_layer.cpp
.. _Range Layer: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/range_sensor_layer.cpp
.. _Inflation Layer: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/inflation_layer.cpp
.. _Obstacle Layer: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/obstacle_layer.cpp
.. _Spatio-Temporal Voxel Layer: https://github.com/SteveMacenski/spatio_temporal_voxel_layer/
.. _Non-Persistent Voxel Layer: https://github.com/SteveMacenski/nonpersistent_voxel_layer
.. _Denoise Layer: https://github.com/ryzhikovas/navigation2/tree/feature-costmap2d-denoise/nav2_costmap_2d/plugins/denoise_layer.cpp

Costmap Filters
===============

+--------------------+--------------------+-----------------------------------+
|    Plugin Name     |      Creator       |       Description                 |
+====================+====================+===================================+
| `Keepout Filter`_  | Alexey Merzlyakov  | Maintains keep-out/safety zones   |
|                    |                    | and preferred lanes for moving    |
+--------------------+--------------------+-----------------------------------+
| `Speed Filter`_    | Alexey Merzlyakov  | Limits maximum velocity of robot  |
|                    |                    | in speed restriction areas        |
+--------------------+--------------------+-----------------------------------+
| `Binary Filter`_   | Alexey Merzlyakov  | Enables binary (boolean) mask     |
|                    |                    | behavior to trigger actions.      |
+--------------------+--------------------+-----------------------------------+

.. _Keepout Filter: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp
.. _Speed Filter: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/costmap_filters/speed_filter.cpp
.. _Binary Filter: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/costmap_filters/binary_filter.cpp

Controllers
===========

+----------------------------+--------------------+----------------------------------+-----------------------+
|      Plugin Name           |       Creator      |       Description                | Drivetrain support    |
+============================+====================+==================================+=======================+
|  `DWB Controller`_         | David Lu!!         | A highly configurable  DWA       | Differential,         |
|                            |                    | implementation with plugin       | Omnidirectional,      |
|                            |                    | interfaces                       | Legged                |
+----------------------------+--------------------+----------------------------------+-----------------------+
|  `TEB Controller`_         | Christoph Rösmann  | A MPC-like controller suitable   | **Ackermann**, Legged,|
|                            |                    | for ackermann, differential, and | Omnidirectional,      |
|                            |                    | holonomic robots.                | Differential          |
+----------------------------+--------------------+----------------------------------+-----------------------+
| `Regulated Pure Pursuit`_  | Steve Macenski     | A service / industrial robot     | **Ackermann**, Legged,|
|                            |                    | variation on the pure pursuit    | Differential          |
|                            |                    | algorithm with adaptive features.|                       |
+----------------------------+--------------------+----------------------------------+-----------------------+
| `MPPI Controller`_         | Steve Macenski     | A predictive MPC controller with | Differential, Omni,   |
|                            | Aleksei Budyakov   | modular & custom cost functions  | **Ackermann**         |
|                            |                    | that can accomplish many tasks.  |                       |
+----------------------------+--------------------+----------------------------------+-----------------------+
| `Rotation Shim Controller`_| Steve Macenski     | A "shim" controller to rotate    | Differential, Omni,   |
|                            |                    | to path heading before passing   | model rotate in place |
|                            |                    | to main controller for  tracking.|                       |
+----------------------------+--------------------+----------------------------------+-----------------------+

.. _DWB Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_dwb_controller
.. _TEB Controller: https://github.com/rst-tu-dortmund/teb_local_planner
.. _Regulated Pure Pursuit: https://github.com/ros-planning/navigation2/tree/main/nav2_regulated_pure_pursuit_controller
.. _Rotation Shim Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_rotation_shim_controller
.. _MPPI Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_mppi_controller

Planners
========

+---------------------------+---------------------------------------+------------------------------+---------------------+
| Plugin Name               |         Creator                       |       Description            | Drivetrain support  |
+===========================+=======================================+==============================+=====================+
|  `NavFn Planner`_         | Eitan Marder-Eppstein & Kurt Konolige | A navigation function        | Differential,       |
|                           |                                       | using A* or Dijkstras        | Omnidirectional,    |
|                           |                                       | expansion, assumes 2D        | Legged              |
|                           |                                       | holonomic particle           |                     |
+---------------------------+---------------------------------------+------------------------------+---------------------+
| `SmacPlannerHybrid`_      | Steve Macenski                        | A SE2 Hybrid-A*              | **Ackermann**,      |
|  (formerly `SmacPlanner`) |                                       | implementation using either  | Differential,       |
|                           |                                       | Dubin or Reeds-shepp motion  | Omnidirectional,    |
|                           |                                       | models with smoother and     | Legged              |
|                           |                                       | multi-resolution query.      |                     |
|                           |                                       | Cars, car-like, and          |                     |
|                           |                                       | ackermann vehicles.          |                     |
|                           |                                       | Kinematically feasible.      |                     |
+---------------------------+---------------------------------------+------------------------------+---------------------+
|  `SmacPlanner2D`_         | Steve Macenski                        | A 2D A* implementation       | Differential,       |
|                           |                                       | Using either 4 or 8          | Omnidirectional,    |
|                           |                                       | connected neighborhoods      | Legged              |
|                           |                                       | with smoother and            |                     |
|                           |                                       | multi-resolution query       |                     |
+---------------------------+---------------------------------------+------------------------------+---------------------+
|  `SmacPlannerLattice`_    | Steve Macenski                        | An implementation of State   | Differential,       |
|                           |                                       | Lattice Planner using        | Omnidirectional,    |
|                           |                                       | pre-generated minimum control| Ackermann,          |
|                           |                                       | sets for kinematically       | Legged,             |
|                           |                                       | feasible planning with any   | Arbitrary / Custom  |
|                           |                                       | type of vehicle imaginable.  |                     |
|                           |                                       | Includes generator script for|                     |
|                           |                                       | Ackermann, diff, omni, and   |                     |
|                           |                                       | legged robots.               |                     |
+---------------------------+---------------------------------------+------------------------------+---------------------+
|`ThetaStarPlanner`_        | Anshumaan Singh                       | An implementaion of Theta*   | Differential,       |
|                           |                                       | using either 4 or 8          | Omnidirectional     |
|                           |                                       | connected neighborhoods,     |                     |
|                           |                                       | assumes the robot as a       |                     |
|                           |                                       | 2D holonomic particle        |                     |
+---------------------------+---------------------------------------+------------------------------+---------------------+

.. _NavFn Planner: https://github.com/ros-planning/navigation2/tree/main/nav2_navfn_planner
.. _SmacPlannerHybrid: https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner
.. _SmacPlanner2D: https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner
.. _ThetaStarPlanner: https://github.com/ros-planning/navigation2/tree/main/nav2_theta_star_planner
.. _SmacPlannerLattice: https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner


Smoothers
=========

+---------------------------+---------------------------------------+------------------------------+
| Plugin Name               |         Creator                       |       Description            |
+===========================+=======================================+==============================+
|  `Simple Smoother`_       | Steve Macenski                        | A simple path smoother for   |
|                           |                                       | infeasible (e.g. 2D)         |
|                           |                                       | planners                     |
+---------------------------+---------------------------------------+------------------------------+
|  `Constrained Smoother`_  | Matej Vargovcik & Steve Macenski      | A path smoother using a      |
|                           |                                       | constraints problem solver   |
|                           |                                       | to optimize various criteria |
|                           |                                       | such as smoothness or        |
|                           |                                       | distance from obstacles,     |
|                           |                                       | maintaining minimum turning  |
|                           |                                       | radius                       |
+---------------------------+---------------------------------------+------------------------------+
|`Savitzky-Golay Smoother`_ |  Steve Macenski                       | A path smoother using a      |
|                           |                                       | Savitzky-Golay filter        |
|                           |                                       | to smooth the path via       |
|                           |                                       | digital signal processing    |
|                           |                                       | to remove noise from the     |
|                           |                                       | path.                        |
+---------------------------+---------------------------------------+------------------------------+

.. _Simple Smoother: https://github.com/ros-planning/navigation2/tree/main/nav2_smoother
.. _Constrained Smoother: https://github.com/ros-planning/navigation2/tree/main/nav2_constrained_smoother
.. _Savitzky-Golay Smoother: https://github.com/ros-planning/navigation2/tree/main/nav2_smoother

Behaviors
=========

+----------------------+------------------------+----------------------------------+
|  Plugin Name         |         Creator        |       Description                |
+======================+========================+==================================+
|  `Clear Costmap`_    | Eitan Marder-Eppstein  | A service to clear the given     |
|                      |                        | costmap in case of incorrect     |
|                      |                        | perception or robot is stuck     |
+----------------------+------------------------+----------------------------------+
|  `Spin`_             | Steve Macenski         | Rotate behavior of configurable  |
|                      |                        | angles to clear out free space   |
|                      |                        | and nudge robot out of potential |
|                      |                        | local failures                   |
+----------------------+------------------------+----------------------------------+
|    `Back Up`_        | Brian Wilcox           | Back up behavior of configurable |
|                      |                        | distance to back out of a        |
|                      |                        | situation where the robot is     |
|                      |                        | stuck                            |
+----------------------+------------------------+----------------------------------+
|             `Wait`_  | Steve Macenski         | Wait behavior with configurable  |
|                      |                        | time to wait in case of time     |
|                      |                        | based obstacle like human traffic|
|                      |                        | or getting more sensor data      |
+----------------------+------------------------+----------------------------------+
|  `Drive On Heading`_ | Joshua Wallace         | Drive on heading behavior with   |
|                      |                        | configurable distance to drive   |
+----------------------+------------------------+----------------------------------+
|  `Assisted Teleop`_  | Joshua Wallace         | AssistedTeleop behavior that     |
|                      |                        | scales teleop commands to        |
|                      |                        | prevent collisions.              |
+----------------------+------------------------+----------------------------------+

.. _Back Up: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors/plugins
.. _Spin: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors/plugins
.. _Wait: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors/plugins
.. _Drive On Heading: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors/plugins
.. _Clear Costmap: https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/src/clear_costmap_service.cpp
.. _Assisted Teleop: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors/plugins

Waypoint Task Executors
=======================

+---------------------------------+------------------------+----------------------------------+
|        Plugin Name              |         Creator        |       Description                |
+=================================+========================+==================================+
| `WaitAtWaypoint`_               | Fetullah Atas          | A plugin to execute a wait       |
|                                 |                        | behavior  on                     |
|                                 |                        | waypoint arrivals.               |
|                                 |                        |                                  |
+---------------------------------+------------------------+----------------------------------+
| `PhotoAtWaypoint`_              | Fetullah Atas          | A plugin to take and save photos |
|                                 |                        | to specified directory on        |
|                                 |                        | waypoint arrivals.               |
|                                 |                        |                                  |
+---------------------------------+------------------------+----------------------------------+
| `InputAtWaypoint`_              | Steve Macenski         | A plugin to wait for user input  |
|                                 |                        | before moving onto the next      |
|                                 |                        | waypoint.                        |
+---------------------------------+------------------------+----------------------------------+

.. _WaitAtWaypoint: https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower/plugins/wait_at_waypoint.cpp
.. _PhotoAtWaypoint: https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower/plugins/photo_at_waypoint.cpp
.. _InputAtWaypoint: https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower/plugins/input_at_waypoint.cpp

Goal Checkers
=============

+---------------------------------+------------------------+----------------------------------+
|     Plugin Name                 |         Creator        |       Description                |
+=================================+========================+==================================+
| `SimpleGoalChecker`_            | David Lu!!             | A plugin check whether robot     |
|                                 |                        | is within translational distance |
|                                 |                        | and rotational distance of goal. |
|                                 |                        |                                  |
+---------------------------------+------------------------+----------------------------------+
| `StoppedGoalChecker`_           | David Lu!!             | A plugin check whether robot     |
|                                 |                        | is within translational distance |
|                                 |                        | , rotational distance of goal,   |
|                                 |                        | and velocity threshold.          |
+---------------------------------+------------------------+----------------------------------+

.. _SimpleGoalChecker: https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/simple_goal_checker.cpp
.. _StoppedGoalChecker: https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/stopped_goal_checker.cpp

Progress Checkers
=================

+---------------------------------+------------------------+----------------------------------+
|         Plugin Name             |         Creator        |       Description                |
+=================================+========================+==================================+
| `SimpleProgressChecker`_        | David Lu!!             | A plugin to check whether the    |
|                                 |                        | robot was able to move a minimum |
|                                 |                        | distance in a given time to      |
|                                 |                        | make progress towards a goal     |
+---------------------------------+------------------------+----------------------------------+
| `PoseProgressChecker`_          | Guillaume Doisy        | A plugin to check whether the    |
|                                 |                        | robot was able to move a minimum |
|                                 |                        | distance or angle in a given time|
|                                 |                        | to make progress towards a goal  |
+---------------------------------+------------------------+----------------------------------+

.. _SimpleProgressChecker: https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/simple_progress_checker.cpp
.. _PoseProgressChecker: https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/pose_progress_checker.cpp


Behavior Tree Nodes
===================

+--------------------------------------------+---------------------+------------------------------------------+
| Action Plugin Name                         |   Creator           |       Description                        |
+============================================+=====================+==========================================+
| `Back Up Action`_                          | Michael Jeronimo    | Calls backup behavior action             |
+--------------------------------------------+---------------------+------------------------------------------+
| `Drive On Heading Action`_                 | Joshua Wallace      | Calls drive on heading behavior action   |
+--------------------------------------------+---------------------+------------------------------------------+
| `Assisted Teleop Action`_                  | Joshua Wallace      | Calls assisted teleop behavior action    |
+--------------------------------------------+---------------------+------------------------------------------+
| `Clear Entire Costmap Service`_            | Carl Delsey         | Calls clear entire costmap service       |
+--------------------------------------------+---------------------+------------------------------------------+
| `Clear Costmap Except Region Service`_     | Guillaume Doisy     | Calls clear costmap except region service|
+--------------------------------------------+---------------------+------------------------------------------+
| `Clear Costmap Around Robot Service`_      | Guillaume Doisy     | Calls clear costmap around robot service |
+--------------------------------------------+---------------------+------------------------------------------+
| `Compute Path to Pose Action`_             | Michael Jeronimo    | Calls Nav2 planner server                |
+--------------------------------------------+---------------------+------------------------------------------+
| `Smooth Path Action`_                      | Matej Vargovcik     | Calls Nav2 smoother server               |
+--------------------------------------------+---------------------+------------------------------------------+
| `Follow Path Action`_                      | Michael Jeronimo    | Calls Nav2 controller server             |
+--------------------------------------------+---------------------+------------------------------------------+
| `Navigate to Pose Action`_                 | Michael Jeronimo    | BT Node for other                        |
|                                            |                     | BehaviorTree.CPP BTs to call             |
|                                            |                     | Navigation2 as a subtree action          |
+--------------------------------------------+---------------------+------------------------------------------+
| `Reinitalize Global Localization Service`_ | Carl Delsey         | Reinitialize AMCL to a new pose          |
+--------------------------------------------+---------------------+------------------------------------------+
| `Spin Action`_                             | Carl Delsey         | Calls spin behavior action               |
+--------------------------------------------+---------------------+------------------------------------------+
| `Wait Action`_                             | Steve Macenski      | Calls wait behavior action               |
+--------------------------------------------+---------------------+------------------------------------------+
| `Truncate Path`_                           | Francisco Martín    | Modifies a path making it shorter        |
+--------------------------------------------+---------------------+------------------------------------------+
| `Truncate Path Local`_                     | Matej Vargovcik     | Extracts a path section around robot     |
+--------------------------------------------+---------------------+------------------------------------------+
| `Planner Selector`_                        | Pablo Iñigo Blasco  | Selects the global planner based on a    |
|                                            |                     | topic input, otherwises uses a default   |
|                                            |                     | planner id                               |
+--------------------------------------------+---------------------+------------------------------------------+
| `Controller Selector`_                     | Pablo Iñigo Blasco  | Selects the controller based on a        |
|                                            |                     | topic input, otherwises uses a default   |
|                                            |                     | controller id                            |
+--------------------------------------------+---------------------+------------------------------------------+
| `Goal Checker Selector`_                   | Pablo Iñigo Blasco  | Selects the goal checker based on a      |
|                                            |                     | topic input, otherwises uses a default   |
|                                            |                     | goal checker id                          |
+--------------------------------------------+---------------------+------------------------------------------+
| `Smoother Selector`_                       | Owen Hooper         | Selects the smoother based on a          |
|                                            |                     | topic input, otherwises uses a default   |
|                                            |                     | smoother id                              |
+--------------------------------------------+---------------------+------------------------------------------+
| `Navigate Through Poses`_                  | Steve Macenski      | BT Node for other BehaviorTree.CPP BTs   |
|                                            |                     | to call Nav2's NavThroughPoses action    |
|                                            |                     |                                          |
+--------------------------------------------+---------------------+------------------------------------------+
| `Remove Passed Goals`_                     | Steve Macenski      | Removes goal poses passed or within a    |
|                                            |                     | tolerance for culling old viapoints from |
|                                            |                     | path re-planning                         |
+--------------------------------------------+---------------------+------------------------------------------+
| `Compute Path Through Poses`_              | Steve Macenski      | Computes a path through a set of poses   |
|                                            |                     | rather than a single end goal pose       |
|                                            |                     | using the planner plugin specified       |
+--------------------------------------------+---------------------+------------------------------------------+
| `Cancel Control Action`_                   |Pradheep Padmanabhan | Cancels Nav2 controller server           |
+--------------------------------------------+---------------------+------------------------------------------+
| `Cancel BackUp Action`_                    |Pradheep Padmanabhan | Cancels backup behavior action           |
+--------------------------------------------+---------------------+------------------------------------------+
| `Cancel Spin Action`_                      |Pradheep Padmanabhan | Cancels spin behavior action             |
+--------------------------------------------+---------------------+------------------------------------------+
| `Cancel Wait Action`_                      |Pradheep Padmanabhan | Cancels wait behavior action             |
+--------------------------------------------+---------------------+------------------------------------------+
| `Cancel Drive on Heading Action`_          | Joshua Wallace      | Cancels drive on heading behavior action |
+--------------------------------------------+---------------------+------------------------------------------+
| `Cancel Assisted Teleop Action`_           | Joshua Wallace      | Cancels assisted teleop behavior action  |
+--------------------------------------------+---------------------+------------------------------------------+

.. _Back Up Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/back_up_action.cpp
.. _Drive On Heading Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/drive_on_heading_action.cpp
.. _Assisted Teleop Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/assisted_teleop_action.cpp
.. _Clear Entire Costmap Service: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
.. _Clear Costmap Except Region Service: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
.. _Clear Costmap Around Robot Service: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
.. _Compute Path to Pose Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp
.. _Smooth Path Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/smooth_path_action.cpp
.. _Follow Path Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/follow_path_action.cpp
.. _Navigate to Pose Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/navigate_to_pose_action.cpp
.. _Reinitalize Global Localization Service: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/reinitialize_global_localization_service.cpp
.. _Spin Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/spin_action.cpp
.. _Wait Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/wait_action.cpp
.. _Truncate Path: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/truncate_path_action.cpp
.. _Truncate Path Local: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/truncate_path_local_action.cpp
.. _Planner Selector: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/planner_selector_node.cpp
.. _Controller Selector: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/controller_selector_node.cpp
.. _Goal Checker Selector: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/goal_checker_selector_node.cpp
.. _Smoother Selector: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/smoother_selector_node.cpp
.. _Navigate Through Poses: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/navigate_through_poses_action.cpp
.. _Remove Passed Goals: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/remove_passed_goals_action.cpp
.. _Compute Path Through Poses: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/compute_path_through_poses_action.cpp
.. _Cancel Control Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/controller_cancel_node.cpp
.. _Cancel BackUp Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/back_up_cancel_node.cpp
.. _Cancel Spin Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/spin_cancel_node.cpp
.. _Cancel Wait Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/wait_cancel_node.cpp
.. _Cancel Drive on Heading Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/drive_on_heading_cancel_node.cpp
.. _Cancel Assisted Teleop Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/assisted_teleop_cancel_node.cpp


+------------------------------------+--------------------+------------------------+
| Condition Plugin Name              |         Creator    |       Description      |
+====================================+====================+========================+
| `Goal Reached Condition`_          | Carl Delsey        | Checks if goal is      |
|                                    |                    | reached within tol.    |
+------------------------------------+--------------------+------------------------+
| `Goal Updated Condition`_          |Aitor Miguel Blanco | Checks if goal is      |
|                                    |                    | preempted.             |
+------------------------------------+--------------------+------------------------+
| `Globally Updated Goal Condition`_ | Joshua Wallace     | Checks if goal is      |
|                                    |                    | preempted in the global|
|                                    |                    | BT context             |
+------------------------------------+--------------------+------------------------+
| `Initial Pose received Condition`_ | Carl Delsey        | Checks if initial pose |
|                                    |                    | has been set           |
+------------------------------------+--------------------+------------------------+
| `Is Stuck Condition`_              |  Michael Jeronimo  | Checks if robot is     |
|                                    |                    | making progress or     |
|                                    |                    | stuck                  |
+------------------------------------+--------------------+------------------------+
| `Transform Available Condition`_   |  Steve Macenski    | Checks if a TF         |
|                                    |                    | transformation is      |
|                                    |                    | available. When        |
|                                    |                    | succeeds returns       |
|                                    |                    | success for subsequent |
|                                    |                    | calls.                 |
+------------------------------------+--------------------+------------------------+
| `Distance Traveled Condition`_     |  Sarthak Mittal    | Checks is robot has    |
|                                    |                    | traveled a given       |
|                                    |                    | distance.              |
+------------------------------------+--------------------+------------------------+
| `Time Expired Condition`_          |  Sarthak Mittal    | Checks if a given      |
|                                    |                    | time period has        |
|                                    |                    | passed.                |
+------------------------------------+--------------------+------------------------+
| `Is Battery Low Condition`_        |  Sarthak Mittal    | Checks if battery      |
|                                    |                    | percentage is below    |
|                                    |                    | a specified value.     |
+------------------------------------+--------------------+------------------------+
| `Is Path Valid Condition`_         |  Joshua Wallace    | Checks if a path is    |
|                                    |                    | valid by making sure   |
|                                    |                    | there are no LETHAL    |
|                                    |                    | obstacles along the    |
|                                    |                    | path.                  |
+------------------------------------+--------------------+------------------------+
| `Path Expiring Timer`_             |  Joshua Wallace    | Checks if the timer has|
|                                    |                    | expired. The timer is  |
|                                    |                    | reset if the path gets |
|                                    |                    | updated.               |
+------------------------------------+--------------------+------------------------+
| `Are Error Codes Present`_         |  Joshua Wallace    | Checks if the specified|
|                                    |                    | error codes are        |
|                                    |                    | present.               |
+------------------------------------+--------------------+------------------------+
| `Would A Controller Recovery Help`_|  Joshua Wallace    | Checks if a controller |
|                                    |                    | recovery could help    |
|                                    |                    | clear the controller   |
|                                    |                    | server error code.     |
+------------------------------------+--------------------+------------------------+
| `Would A Planner Recovery Help`_   |  Joshua Wallace    | Checks if a planner    |
|                                    |                    | recovery could help    |
|                                    |                    | clear the planner      |
|                                    |                    | server error code.     |
+------------------------------------+--------------------+------------------------+
| `Would A Smoother Recovery Help`_  |  Joshua Wallace    | Checks if a Smoother   |
|                                    |                    | recovery could help    |
|                                    |                    | clear the smoother     |
|                                    |                    | server error code.     |
+------------------------------------+--------------------+------------------------+
| `Is Battery Charging Condition`_   |  Alberto Tudela    | Checks if the battery  |
|                                    |                    | is charging.           |
+------------------------------------+--------------------+------------------------+

.. _Goal Reached Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/goal_reached_condition.cpp
.. _Goal Updated Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/goal_updated_condition.cpp
.. _Globally Updated Goal Condition: https://github.com/navigation2/blob/replanning/nav2_behavior_tree/plugins/condition/globally_updated_goal_condition.cpp
.. _Initial Pose received Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/initial_pose_received_condition.cpp
.. _Is Stuck Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/is_stuck_condition.cpp
.. _Transform Available Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/transform_available_condition.cpp
.. _Distance Traveled Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/distance_traveled_condition.cpp
.. _Time Expired Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/time_expired_condition.cpp
.. _Is Battery Low Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/is_battery_low_condition.cpp
.. _Is Path Valid Condition: https://github.com/navigation2/blob/replanning/nav2_behavior_tree/plugins/condition/is_path_valid_condition.cpp
.. _Path Expiring Timer: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.cpp
.. _Are Error Codes Present: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/are_error_codes_present_condition.cpp
.. _Would A Controller Recovery Help: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/would_a_controller_recovery_help.cpp
.. _Would A Planner Recovery Help: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/would_a_planner_recovery_help.cpp
.. _Would A Smoother Recovery Help: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/would_a_smoother_recovery_help.cpp
.. _Is Battery Charging Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/is_battery_charging_condition.cpp

+--------------------------+---------------------+----------------------------------+
| Decorator Plugin Name    |    Creator          |       Description                |
+==========================+=====================+==================================+
| `Rate Controller`_       | Michael Jeronimo    | Throttles child node to a given  |
|                          |                     | rate                             |
+--------------------------+---------------------+----------------------------------+
| `Distance Controller`_   | Sarthak Mittal      | Ticks child node based on the    |
|                          |                     | distance traveled by the robot   |
+--------------------------+---------------------+----------------------------------+
| `Speed Controller`_      | Sarthak Mittal      | Throttles child node to a rate   |
|                          |                     | based on current robot speed.    |
+--------------------------+---------------------+----------------------------------+
| `Goal Updater`_          | Francisco Martín    | Updates the goal received via    |
|                          |                     | topic subscription.              |
+--------------------------+---------------------+----------------------------------+
| `Single Trigger`_        | Steve Macenski      | Triggers nodes/subtrees below    |
|                          |                     | only a single time per BT run.   |
+--------------------------+---------------------+----------------------------------+
| `PathLongerOnApproach`_  | Pradheep Padmanabhan| Triggers child nodes if the new  |
|                          |                     | global path is significantly     |
|                          |                     | larger than the old global path  |
|                          |                     | on approach to the goal          |
+--------------------------+---------------------+----------------------------------+

.. _Rate Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/rate_controller.cpp
.. _Distance Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/distance_controller.cpp
.. _Speed Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/speed_controller.cpp
.. _Goal Updater: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/goal_updater_node.cpp
.. _Single Trigger: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/single_trigger_node.cpp
.. _PathLongerOnApproach: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/path_longer_on_approach.cpp

+-----------------------+------------------------+----------------------------------+
| Control Plugin Name   |         Creator        |       Description                |
+=======================+========================+==================================+
| `Pipeline Sequence`_  | Carl Delsey            | A variant of a sequence node that|
|                       |                        | will re-tick previous children   |
|                       |                        | even if another child is running |
+-----------------------+------------------------+----------------------------------+
| `Recovery`_           | Carl Delsey            | Node must contain 2 children     |
|                       |                        | and returns success if first     |
|                       |                        | succeeds. If first fails, the    |
|                       |                        | second will be ticked. If        |
|                       |                        | successful, it will retry the    |
|                       |                        | first and then return its value  |
+-----------------------+------------------------+----------------------------------+
| `Round Robin`_        | Mohammad Haghighipanah | Will tick ``i`` th child until   |
|                       |                        | a result and move on to ``i+1``  |
+-----------------------+------------------------+----------------------------------+

.. _Pipeline Sequence: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/control/pipeline_sequence.cpp
.. _Recovery: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/control/recovery_node.cpp
.. _Round Robin: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/control/round_robin_node.cpp
