.. _roadmap:

Roadmaps
########

This is the list of major issues and features the Nav2 maintainers are committing for completion for various ROS 2 releases.
This is *not* an exhaustive list of planned features or what changes may be found in the new distribution.
It represents only the items of direct commitment to give insight into commitments for REP-2005 repositories in the `ROS 2 Roadmap <https://docs.ros.org/en/rolling/Roadmap.html>`_.
For a full list of important completed changes in the project, see the Migration Guides :ref:`migration`.


Lyrical Roadmap
***************

+--------------------------------+------------------------+
| Controller Server and Plugin   |  Large (In Progress)   |
| factor for centralized path    |                        |
| handling and crosstrack error  |                        |
| estimation and enforcement.    |                        |
+--------------------------------+------------------------+
| Massive refactor of ``rclcpp`` |  Large (in progress)   |
| types to ``nav2`` types for    |                        |
| ROS API abstraction            |                        |
+--------------------------------+------------------------+
| Improve dynamic feasibility    |  Medium (DONE)         |
| on approach to goal on all     |                        |
| control and behavior plugins   |                        |
+--------------------------------+------------------------+
| Introduce Vector Object Server |  Medium  (DONE)        |
| to augment Costmap Filters     |                        |
| with polygons rather than      |                        |
| annotated masks                |                        |
+--------------------------------+------------------------+
| `Continued Route Server`_      |  Medium                |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| Add Pause and Resume feature   | Medium (DONE)          |
| to Nav2 behavior tree tasks    |                        |
+--------------------------------+------------------------+
| Use PointCloud Transport for   |  Medium (DONE)         |
| all pointcloud subscriptions   |                        |
+--------------------------------+------------------------+
| Update and refine behavior     |  Medium                |
| trees for more intelligent     |                        |
| behavior                       |                        |
+--------------------------------+------------------------+
| Fix for statics in Smac        |  Medium (DONE)         |
| Planner to allow for multiple  |                        |
| instances in a server not to   |                        |
| conflict                       |                        |
+--------------------------------+------------------------+
| Create initial prototype of    |  Large                 |
| a framework for environmental  |                        |
| modeling, sensor processing,   |                        |
| and external model integration |                        |
+--------------------------------+------------------------+

.. _Continued Route Server: https://github.com/ros-navigation/navigation2/issues/5082


Kilted Roadmap
**************

+--------------------------------+------------------------+
| Release of Route server        |   Large (DONE)         |
+--------------------------------+------------------------+
| Allow non-orientation sp. for  |  Medium (DONE)         |
| smac planner goals             |                        |
+--------------------------------+------------------------+
| Allow docking server to operate|  Medium (DONE)         |
| forward and backwards          |                        |
+--------------------------------+------------------------+

Jazzy Roadmap
*************

+--------------------------------+------------------------+
| `Smac Planner Improvements`_   |  Large (DONE)          |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| Get CI Green Again             |  Small (DONE)          |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
|  Various MPPI Improvements     |  Medium (DONE)         |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `Fuse Migration`_              |  Medium  (incomplete)  |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `Ignition Migration`_          |  Medium (DONE)         |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `Route Graph Planner`_         | Large (in progress)    |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| Provide Advanced Capabilities  | Medium (DONE)          |
| Tutorials (e.g. gps, vio)      |                        |
|                                |                        |
+--------------------------------+------------------------+
| ROS Time Respect Across Stack  |  Medium (DONE)         |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| TwistStamped Migration         |  Medium (DONE)         |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| Velocity-Scheduled Collision   |  Medium (DONE)         |
| Monitor Polygons               |                        |
|                                |                        |
+--------------------------------+------------------------+
| Nav2 auto-docking capability   |  Medium (DONE)         |
|                                |                        |
+--------------------------------+------------------------+

Iron Roadmap
************

+--------------------------------+------------------------+
|            Plugin Name         |         Size           |
+================================+========================+
| `Pluginize Navigators`_        | Medium  (DONE)         |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `MPPI Controller`_             | Very Large (DONE)      |
|                                |                        |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `Route Graph Planner`_         | Large (incomplete)     |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| 90% unit test coverage         | Small  (DONE)          |
|                                |                        |
+--------------------------------+------------------------+
|   `Velocity Smoother`_         |  Medium (DONE)         |
+--------------------------------+------------------------+
| ROS Time Respect Across Stack  |  Medium (incomplete)   |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+

.. _Smac Planner Improvements: https://github.com/ros-navigation/navigation2/issues/3172
.. _Pluginize Navigators: https://github.com/ros-navigation/navigation2/issues/3335
.. _MPPI Controller: https://github.com/ros-navigation/navigation2/pull/3350
.. _Route Graph Planner: https://github.com/ros-navigation/navigation2/issues/2229
.. _Velocity Smoother: https://github.com/ros-navigation/navigation2/pull/2964
.. _Fuse Migration: https://github.com/ros-navigation/navigation2/issues/2598
.. _Ignition Migration: https://github.com/ros-navigation/navigation2/issues/2997

Humble Roadmap
**************

+--------------------------------+------------------------+
|            Plugin Name         |         Size           |
+================================+========================+
| `Nav2 1 Node Per Server`_      | Medium  (DONE)         |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `Smac Lattice Planner`_        | Large (DONE)           |
|                                |                        |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `Safety Collision Nodes`_      | Medium (DONE)          |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `Fix Min Range Bug`_           | Small  (DONE)          |
|                                |                        |
+--------------------------------+------------------------+
|   `Move Development            | Small (DONE)           |
|   from Master to Rolling`_     |                        |
|                                |                        |
+--------------------------------+------------------------+
| Push Test Coverage to 88\%     |  Medium (DONE)         |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `Complete First Time Guide`_   |  Medium (DONE)         |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `Rotation Shim Controller`_    |  Small (DONE)          |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+
| `Dynamic Composition`_         |  Medium (DONE)         |
|                                |                        |
|                                |                        |
+--------------------------------+------------------------+

.. _Smac Lattice Planner: https://github.com/ros-navigation/navigation2/issues/1710
.. _Nav2 1 Node Per Server: https://github.com/ros-navigation/navigation2/issues/816
.. _Safety Collision Nodes: https://github.com/ros-navigation/navigation2/issues/1899
.. _Fix Min Range Bug: https://github.com/ros-navigation/navigation2/pull/2460
.. _Complete First Time Guide: https://github.com/ros-navigation/navigation2/issues/1589
.. _Rotation Shim Controller: https://github.com/ros-navigation/navigation2/pull/2718
.. _Move Development from Master to Rolling: https://github.com/ros-navigation/navigation2/issues/2337
.. _Dynamic Composition: https://github.com/ros-navigation/navigation2/issues/2147
