# Roadmaps

This is the list of major issues and features the Nav2 maintainers are committing for completion for various ROS 2 releases.
This is *not* an exhaustive list of planned features or what changes may be found in the new distribution.
It represents only the items of direct commitment to give insight into commitments for REP-2005 repositories in the [ROS 2 Roadmap](https://docs.ros.org/en/rolling/Roadmap.html).
For a full list of important completed changes in the project, see the Migration Guides [Migration Guides](../migration/index.md#migration-guides).

## Lyrical Roadmap

- [ ] (Large) (In Progress) - Controller Server and Plugin factor for centralized path handling and crosstrack error estimation and enforcement.
- [ ] (Large) (In Progress) - Massive refactor of `rclcpp` types to `nav2` types for ROS API abstraction.
- [x] (Medium) - Improve dynamic feasibility on approach to goal on all control and behavior plugins.
- [x] (Medium) - Introduce Vector Object Server to augment Costmap Filters with polygons rather than annotated masks.
- [ ] (Medium) - [Continued Route Server][]
- [x] (Medium) - Add Pause and Resume feature to Nav2 behavior tree tasks.
- [x] (Medium) - Use PointCloud Transport for all pointcloud subscriptions.
- [ ] (Medium) - Update and refine behavior trees for more intelligent behavior.
- [x] (Medium) - Fix for statics in Smac Planner to allow for multiple instances in a server not to conflict.
- [ ] (Large) - Create initial prototype of a framework for environmental modeling, sensor processing, and external model integration.

[Continued Route Server]: https://github.com/ros-navigation/navigation2/issues/5082

## Kilted Roadmap

- [x] (Large) - Release of Route server.
- [x] (Medium) - Allow non-orientation sp. for smac planner goals.
- [x] (Medium) - Allow docking server to operate forward and backwards.

## Jazzy Roadmap

- [x] (Large) - [Smac Planner Improvements][]
- [x] (Small) - Get CI Green Again.
- [x] (Medium) - Various MPPI Improvements.
- [ ] (Medium) (Incomplete) - [Fuse Migration][]
- [x] (Medium) - [Ignition Migration][]
- [ ] (Large) (In Progress) - [Route Graph Planner][]
- [x] (Medium) - Provide Advanced Capabilities Tutorials (e.g. gps, vio).
- [x] (Medium) - ROS Time Respect Across Stack.
- [x] (Medium) - TwistStamped Migration.
- [x] (Medium) - Velocity-Scheduled Collision Monitor Polygons.
- [x] (Medium) - Nav2 auto-docking capability.

[Smac Planner Improvements]: https://github.com/ros-navigation/navigation2/issues/3172
[Fuse Migration]: https://github.com/ros-navigation/navigation2/issues/2598
[Ignition Migration]: https://github.com/ros-navigation/navigation2/issues/2997
[Route Graph Planner]: https://github.com/ros-navigation/navigation2/issues/2229

## Iron Roadmap

- [x] (Medium) - [Pluginize Navigators][]
- [x] (Very Large) - [MPPI Controller][]
- [ ] (Large) (Incomplete) - [Route Graph Planner][]
- [x] (Small) - 90% unit test coverage.
- [x] (Medium) - [Velocity Smoother][]
- [ ] (Medium) (Incomplete) - ROS Time Respect Across Stack.

[Pluginize Navigators]: https://github.com/ros-navigation/navigation2/issues/3335
[MPPI Controller]: https://github.com/ros-navigation/navigation2/pull/3350
[Route Graph Planner]: https://github.com/ros-navigation/navigation2/issues/2229
[Velocity Smoother]: https://github.com/ros-navigation/navigation2/pull/2964

## Humble Roadmap

- [x] (Medium) - [Nav2 1 Node Per Server][]
- [x] (Large) - [Smac Lattice Planner][]
- [x] (Medium) - [Safety Collision Nodes][]
- [x] (Small) - [Fix Min Range Bug][]
- [x] (Small) - [Move Development from Master to Rolling][]
- [x] (Medium) - Push Test Coverage to 88%.
- [x] (Medium) - [Complete First Time Guide][]
- [x] (Small) - [Rotation Shim Controller][]
- [x] (Medium) - [Dynamic Composition][]

[Nav2 1 Node Per Server]: https://github.com/ros-navigation/navigation2/issues/816
[Smac Lattice Planner]: https://github.com/ros-navigation/navigation2/issues/1710
[Safety Collision Nodes]: https://github.com/ros-navigation/navigation2/issues/1899
[Fix Min Range Bug]: https://github.com/ros-navigation/navigation2/pull/2460
[Move Development from Master to Rolling]: https://github.com/ros-navigation/navigation2/issues/2337
[Complete First Time Guide]: https://github.com/ros-navigation/navigation2/issues/1589
[Rotation Shim Controller]: https://github.com/ros-navigation/navigation2/pull/2718
[Dynamic Composition]: https://github.com/ros-navigation/navigation2/issues/2147
