.. _humble_migration:

Humble to Iron
##############

Moving from ROS 2 Humble to Iron, a number of stability improvements were added that we will not specifically address here.

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

Expaned Planner Benchmark Tests
*******************************

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
`PR #3283 <https://github.com/ros-planning/navigation2/pull/3283>`_ adds a BT node to set the smoother based on a topic or a default. See the configuration guide :ref:`SimpleSmoother` for more details. 

Publish Costmap Layers 
**********************
`PR #3320 <https://github.com/ros-planning/navigation2/pull/3320>`_ adds the ability for the nav2_costmap_2d package to publish out costmap data associated with each layer.