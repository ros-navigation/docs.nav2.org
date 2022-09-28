.. _humble_migration

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

`PR #3146 <https://github.com/ros-planning/navigation2/pull/3146>`_ updates the global planners to throw exceptions on planning failures. These exceptions get reported back to the planner server which in turn places a error code on the blackboard. 