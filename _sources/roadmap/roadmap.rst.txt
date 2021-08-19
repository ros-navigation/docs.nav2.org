.. _roadmap:

Humble Roadmap
##############

This is the list of major issues and features the Nav2 maintainers are commiting for completion for the ROS 2 Humble Release in 2022.
This is *not* an exhaustive list of planned features or what changes may be found in the new distribution.
It represents only the items of direct commitment to give insight into commitments for REP-2005 repositories in the `ROS 2 Roadmap <https://docs.ros.org/en/rolling/Roadmap.html>`_.
For a full list of important completed changes in the project, see the Migration Guide :ref:`galactic_migration`.

+--------------------------------+------------------------+----------------------------------+
|            Plugin Name         |         Size           |       Description                |
+================================+========================+==================================+
| `Nav2 1 Node Per Server`_      | Medium                 | Maintains persistant             |
|                                |                        | put data from sensors that       |
|                                |                        | publish range msgs on the costmap|
+--------------------------------+------------------------+----------------------------------+
| `Smac Lattice Planner`_        | Very Large             | Maintains persistant             |
|                                |                        | 3D voxel layer using depth and   |
|                                |                        | laser sensor readings and        |
|                                |                        | raycasting to clear free space   |
+--------------------------------+------------------------+----------------------------------+
| `Safety Collision Nodes`_      | Medium                 | Maintains persistant             |
|                                |                        | occupancy information into       |
|                                |                        | costmap                          |
+--------------------------------+------------------------+----------------------------------+
| `Fix Min Range Bug`_           | Small                  | Maintains persistant             |
|                                |                        | costmap with exponential decay   |
+--------------------------------+------------------------+----------------------------------+
|   `Move Development            | Small                  | Maintains persistent 2D costmap  |
|   from Master to Rolling`_     |                        | from 2D laser scans with         |
|                                |                        | raycasting to clear free space   |
+--------------------------------+------------------------+----------------------------------+
| Push Test Coverage to 88\%     |  Medium                | Maintains temporal 3D sparse     |
|                                |                        | volumetric voxel grid with decay |
|                                |                        | through sensor models            |
+--------------------------------+------------------------+----------------------------------+
| `Complete First Time Guide`_   |  Medium                | Maintains 3D occupancy grid      |
|                                |                        | consisting only of the most      |
|                                |                        | sets of measurements             |
+--------------------------------+------------------------+----------------------------------+

.. _Smac Lattice Planner: https://github.com/ros-planning/navigation2/issues/1710
.. _Nav2 1 Node Per Server: https://github.com/ros-planning/navigation2/issues/816
.. _Safety Collision Nodes: https://github.com/ros-planning/navigation2/issues/1899
.. _Fix Min Range Bug: https://github.com/ros-planning/navigation2/pull/2460
.. _Complete First Time Guide: https://github.com/ros-planning/navigation2/issues/1589
.. _Move Development from Master to Rolling: https://github.com/ros-planning/navigation2/issues/2337
