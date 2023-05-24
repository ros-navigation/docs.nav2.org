.. _humble_migration:

Humble to Iron
##############

Moving from ROS 2 Humble to Iron, a number of stability improvements were added that we will not specifically address here.

New Behavior-Tree Navigator Plugins
***********************************

New in `PR 3345 <https://github.com/ros-planning/navigation2/pull/3345>`_, the navigator types are exposed to users as plugins that can be replaced or new navigator types added. The default behaviors of navigate to pose and navigate through poses continue to be default behavior but are now customizable with new action interface definitions. These plugins implement the ``nav2_core::BehaviorTreeNavigator`` base class, which must process the action request, feedback, and completion messages. The behavior tree is handled by this base class with as much general logic as possible abstracted away from users to minimize repetition.

See :ref:`writing_new_nav2navigator_plugin` for a tutorial about writing new navigator plugins.

Added Collision Monitor
***********************
`PR 2982 <https://github.com/ros-planning/navigation2/pull/2982>`_ adds new safety layer operating independently of Nav2 stack which ensures the robot to control the collisions with near obstacles, obtained from different sensors (LaserScan, PointCloud, IR, Sonars, etc...). See :ref:`configuring_collision_monitor` for more details. It is not included in the default bringup batteries included from ``nav2_bringup``.

Removed use_sim_time from yaml
******************************
`PR #3131 <https://github.com/ros-planning/navigation2/pull/3131>`_ makes it possible to set the use_sim_time parameter from the launch file for multiple nodes instead of individually via the yaml files. If using the Nav2 launch files, you can optionally remove the use_sim_time parameter from your yaml files and set it via a launch argument.

Run-time Speed up of Smac Planner
*********************************
The core data structure of the graph implementation in the Smac Planner framework was swapped out in `PR 3201 <https://github.com/ros-planning/navigation2/pull/3201>`_ to using a specialized unordered map implementation. This speeds up the planner by 10% on trivial requests and reports up to 30% on complex plans that involve numerous rehashings.

Recursive Refinement of Smac and Simple Smoothers
*************************************************

The number of recursive refinements for the Simple and Smac Planner Smoothers have been exposed under the ``refinement_num`` parameter. Previous behavior had this hardcoded if ``do_refinement = True`` to ``4``. Now, default is ``2`` to help decrease out-of-the-box over smoothing reducing in paths closer to collision than probably ideal, but old behavior can be achieved by changing this to ``4``.

Simple Commander Python API
***************************
`PR 3159 <https://github.com/ros-planning/navigation2/pull/3159>`_ and follow-up PRs add in Costmap API in Python3 simple commander to receive ``OccupancyGrid`` messages from Nav2 and be able to work with them natively in Python3, analog to the C++ Costmap API. It also includes a line iterator and collision checking object to perform footprint or other collision checking in Python3. See the Simple Commander API for more details.

Smac Planner Start Pose Included in Path
****************************************

`PR 3168 <https://github.com/ros-planning/navigation2/pull/3168>`_ adds the starting pose to the Smac Planners that was previously excluded during backtracing.

Parameterizable Collision Checking in RPP
*****************************************

`PR 3204 <https://github.com/ros-planning/navigation2/pull/3204>`_ adds makes collision checking for RPP optional (default on).

Expanded Planner Benchmark Tests
********************************

`PR 3218 <https://github.com/ros-planning/navigation2/pull/3218>`_ adds launch files and updated scripts for performing objective random planning tests across the planners in Nav2 for benchmarking and metric computation.

Smac Planner Path Tolerances
****************************

`PR 3219 <https://github.com/ros-planning/navigation2/pull/3219>`_ adds path tolerances to Hybrid-A* and State Lattice planners to return approximate paths if exact paths cannot be found, within a configurable tolerance aroun the goal pose.

costmap_2d_node default constructor
***********************************

`PR #3222 <https://github.com/ros-planning/navigation2/pull/3222>`_ changes the constructor used by the standalone costmap node. The new constructor does not set a name and namespace internally so it can be set via the launch file.

Feedback for Navigation Failures
********************************

`PR #3146 <https://github.com/ros-planning/navigation2/pull/3146>`_ updates the global planners to throw exceptions on planning failures. These exceptions get reported back to the planner server which in turn places a error code on the Behavior Tree Navigator's blackboard for use in contextual error handling in the autonomy application.

The following errors codes are supported (with more to come as necessary): Unknown, TF Error, Start or Goal Outside of Map, Start or Goal Occupied, Timeout, or No Valid Path Found.

`PR #3248 <https://github.com/ros-planning/navigation2/pull/3248>`_ updates the compute path through poses action to report planning failures. These exceptions get reported back to the planner server which in turn places a error code on the Behavior Tree Navigator's blackboard for use in contextual error handling in the autonomy application.

The following errors codes are supported (with more to come as necessary): Unknown, TF Error, Start or Goal Outside of Map, Start or Goal Occupied, Timeout, No Valid Path Found and No Waypoints given.

`PR #3227 <https://github.com/ros-planning/navigation2/pull/3227>`_ updates the controllers to throw exceptions on failures. These exceptions get reported back to the controller server which in turn places a error code on the Behavior Tree Navigatior's blackboard for use in contextual error handling in the autonomy application.

The following error codes are supported (with more to come as necessary): Unknown, TF Error, Invalid Path, Patience Exceeded, Failed To Make Progress, or No Valid Control.

`PR #3251 <https://github.com/ros-planning/navigation2/pull/3251>`_ pipes the highest priority error code through the bt_navigator and defines the error code structure. 

A new parameter for the the BT Navigator called "error_code_id_names" was added to the nav2_params.yaml to define the error codes to compare. 
The lowest error in the "error_code_id_names" is then returned in the action request (navigate to pose, navigate through poses waypoint follower), whereas the code enums increase the higher up in the software stack - giving higher priority to lower-level failures.

The error codes produced from the servers follow the guidelines stated below. 
Error codes from 0 to 9999 are reserved for nav2 while error codes from 10000-65535 are reserved for external servers. 
Each server has two "reserved" error codes. 0 is reserved for NONE and the first error code in the sequence is reserved for UNKNOWN.

The current implemented servers with error codes are:

- Controller Server: NONE:0, UNKNOWN:100, server error codes: 101-199
- Planner Server(compute_path_to_pose): NONE:0, UNKNOWN:201, server error codes: 201-299
- Planner Server(compute_path_through_poses): NONE:0, UNKNOWN:301, server error codes: 301-399
- Smoother Server: NONE: 0, UNKNOWN: 501, server error codes: 501-599
- Waypoint Follower Server: NONE: 0, UNKNOWN: 601, server error codes: 601-699

This pr also updates the waypoint follower server to throw exceptions on failures. These exceptions get reported back to the server which in turn places a error code on the Behavior Tree Navigator's blackboard for use in contextual error handling in the autonomy application.
The following errors codes are supported (with more to come as necessary): Unknown and Task Executor Failed.
See :ref:`adding_a_nav2_task_server` and the PR for additional information.

Costmap Filters
***************

Costmap Filters now are have an ability to be enabled/disabled in run-time by calling ``toggle_filter`` service for appropriate filter (`PR #3229 <https://github.com/ros-planning/navigation2/pull/3229>`_).

Added new binary flip filter, allowing e.g. to turn off camera in sensitive areas, turn on headlights/leds/other safety things or switch operating mode when robot is inside marked on mask areas (`PR #3228 <https://github.com/ros-planning/navigation2/pull/3228>`_).

Savitzky-Golay Smoother
***********************

Adding a new smoother algorithm, the Savitzky-Golay smoother to the smoother server plugin list. See the configuration guide :ref:`configuring_savitzky_golay_filter_smoother` for more details.

Changes to Map yaml file path for map_server node in Launch
***********************************************************
`PR #3174 <https://github.com/ros-planning/navigation2/pull/3174>`_ adds a way to set the path to map yaml file for the map_server node either from the yaml file or using the launch configuration parameter ``map`` giving priority to the launch configuration parameter. ``yaml_filename`` is no longer strictly required to be present in ``nav2_params.yaml``.

SmootherSelector BT Node
************************
`PR #3283 <https://github.com/ros-planning/navigation2/pull/3283>`_ adds a BT node to set the smoother based on a topic or a default. See the configuration guide :ref:`configuring_simple_smoother` for more details. 


Publish Costmap Layers 
**********************
`PR #3320 <https://github.com/ros-planning/navigation2/pull/3320>`_ adds the ability for the nav2_costmap_2d package to publish out costmap data associated with each layer.

Give Behavior Server Access to Both Costmaps
********************************************
`PR #3255 <https://github.com/ros-planning/navigation2/pull/3255>`_ addes the ability for a behavior to access the local and global costmap. 

To update behaviors, any reference to the global_frame must be updated to the local_frame parameter
along with the ``configuration`` method which now takes in the local and global collision checkers.
Lastly, ``getResourceInfo`` must be overriden to return ``CostmapInfoType::LOCAL``. Other options include ``GLOBAL`` if the behavior useses global costmap and/or footprint)
or ``BOTH`` if both are required. This allows us to only create and maintain the minimum amount of expensive resources.   

New Model Predictive Path Integral Controller
*********************************************

The new Nav2 MPPI Controller is a predictive controller - a successor to TEB and pure path tracking MPC controllers - with Nav2. It uses a sampling based approach to select optimal trajectories, optimizing between successive iterations. It contains plugin-based objective functions for customization and extension for various behaviors and behavioral attributes.

See the README.md and :ref:`configuring_mppic` page for more detail.

Behavior Tree Uses Error Codes
******************************
`PR #3324 <https:https://github.com/ros-planning/navigation2/pull/3324>`_ adds three new condition nodes to check for error codes on the blackboard set by action BT nodes which contain them. 

The ``AreErrorCodesPresent`` condition node allows the user to specify the error code from the server along with the error codes to match against. 
The ``WouldAControllerRecoveryHelp`` checks if the active error code is UNKNOWN, PATIENCE_EXCEEDED, FAILED_TO_MAKE_PROGRESS or NO_VALID_CONTROL. 
If the error code is a match, the condition returns ``SUCCESS``.
These error code are potentially able to be cleared by a controller recovery. 

The ``WouldAPlannerRecoveryHelp`` checks if the active error code is UNKNOWN, NO_VALID_CONTROL, or TIMEOUT.
If the error code is a match, the condition returns ``SUCCESS``.
These error code are potentially able to be cleared by a planner recovery. 

The ``WouldASmootherRecoveryHelp`` checks if the active error code is UNKNOWN, TIMEOUT, FAILED_TO_SMOOTH_PATH, or SMOOTHED_PATH_IN_COLLISION.
If the error code is a match, the condition returns ``SUCCESS``.
These error code are potentially able to be cleared by a smoother recovery. 

Load, Save and Loop Waypoints from the Nav2 Panel in RViz
*********************************************************

`PR #3165 <https:https://github.com/ros-planning/navigation2/pull/3165>`_ provides three new functionalities for the nav2 panel in RViz, they are:

- load and save waypoints in a yaml file for waypoint following (initial pose can also be stored if required)
- loop functionality to revisit the waypoints
- pause and resume button for stopping and continuing through the waypoints

Looping functionality is not specific to the nav2 panel in RViz. Users utilizing nav2_waypoint_follower can take advantage of the changes made to the FollowWaypoint action, by specifying the desired number of loops in the action request that will be eventually sent to the nav2_waypoint_follower server.

DWB Forward vs Reverse Pruning
******************************

`PR #3374 <https://github.com/ros-planning/navigation2/pull/3374>`_ adds a new ``forward_prune_distance`` parameter in the DWB controller. It replaces the ``prune_distance`` for forward path shortening, enabled through the ``shorten_transformed_plan`` boolean parameter. This change allows to use different values for forward and backward path shortening.

More stable regulation on curves for long lookahead distances
*************************************************************

`PR #3414 <https://github.com/ros-planning/navigation2/pull/3414>`_ adds a new ``use_fixed_curvature_lookahead`` parameter to the RPP controller. This makes slowing down on curve not dependent on the instantaneous lookahead point, but instead on a fixed distance set by the parameter ``curvature_lookahead_dist``.

Publish Collision Monitor State
*******************************

`PR #3504 <https://github.com/ros-planning/navigation2/pull/3504>`_ adds a new ``state_topic`` parameter to the CollisionMonitor. If specified, this optional parameter enables the state topic publisher. The topic reports the currently activated polygon action type and name.

Renamed ROS-parameter in Collision Monitor
******************************************

`PR #3513 <https://github.com/ros-planning/navigation2/pull/3513>`_ renames ``max_points`` parameter to ``min_points`` and changes its meaning. Formerly ``max_points`` meant the maximum number of points inside the area still not triggering the action, while ``min_points`` - is a minimal number of points starting from the action to be initiated. In other words ``min_points`` now should be adjusted as ``max_points + 1``.

New safety behavior model "limit" in Collision Monitor
******************************************************
`PR #3519 <https://github.com/ros-planning/navigation2/pull/3519>`_ adds a new collision monitor behavior model ``limit`` that restricts maximum linear and angular speed to specific values (``linear_limit`` and ``angular_limit``) if enough points are in the given shape.

Velocity smoother applies deceleration when timeout
***************************************************

`PR #3512 <https://github.com/ros-planning/navigation2/pull/3512>`_ makes the VelocitySmoother apply the deceleration when the input command timeout.

PoseProgressChecker plugin
**************************
`PR #3530 <https://github.com/ros-planning/navigation2/pull/3530>`_ adds a new ``nav2_controller::PoseProgressChecker`` plugin. It builds on the behavior of the ``SimpleProgressChecker`` by adding a new parameter ``required_movement_angle``, allowing the plugin to considers that there is still progress when there is no translation movement, from the moment there is a rotation movement superior to ``required_movement_angle`` within the ``movement_time_allowance``.

Allow multiple goal checkers and change parameter progress_checker_plugin(s) name and type
******************************************************************************************
`PR #3555 <https://github.com/ros-planning/navigation2/pull/3555>`_  initializes the progress checker plugin(s) in the same way as for the goal checker and controller plugins: it is now a list of string and was renamed from ``progress_checker_plugin`` to ``progress_checker_plugins``, and the type changed from ``string`` to ``vector<string>``. This allows the initialization of multiple progress checkers that can be chosen from the added ``progress_checker_id field`` of the ``FollowPath`` action.
Beware that it is a breaking change and that configuration files will need to be updated.

IsBatteryChargingCondition BT Node
**********************************
`PR #3553 <https://github.com/ros-planning/navigation2/pull/3553>`_ adds a BT node to check if the battery is charging. See the configuration guide :ref:`bt_is_battery_charging_condition` for more details. 

Behavior Server Error Codes 
***************************
`PR #3569 <https://github.com/ros-planning/navigation2/pull/3539>`_ updates the behavior server plugins to provide error codes on failure. 

- Spin: NONE: 0, UNKNOWN: 701, server error codes: 701-709
- BackUp: NONE: 0, UNKNOWN: 801, server error codes: 710-719
- DriveOnHeading: NONE: 0, UNKNOWN: 901, server error codes: 720-729
- AssistedTeleop: NONE: 0, UNKNOWN: 1001, server error codes: 730-739

New Denoise Costmap Layer Plugin
********************************
`PR #2567 <https://github.com/ros-planning/navigation2/pull/2567>`_ adds the new plugin for filtering noise on the costmap.

Due to errors in ``Voxel Layer`` or ``Obstacle Layer`` measurements, salt and pepper noise may appear on the :ref:`costmap <configuring_cosmaps>`. This noise creates false obstacles that prevent the robot from finding the best path on the map.
The new ``Denoise Layer`` plugin is designed to filter out noise-induced standalone obstacles or small obstacles groups. This plugin allows you to add layer that will filter local or global costmap.
More information about ``Denoise Layer`` plugin and how it works could be found :ref:`here <filtering_of_noise-induced_obstacles>`.

SmacPlannerHybrid viz_expansions parameter
******************************************
`PR #3577 <https://github.com/ros-planning/navigation2/pull/3577>`_ adds a new paremeter for visualising SmacPlannerHybrid expansions for debug purpose.
