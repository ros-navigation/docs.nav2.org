# Roadmaps { #roadmaps }

This is the list of major issues and features the Nav2 maintainers are committing for completion for various ROS 2 releases.
This is *not* an exhaustive list of planned features or what changes may be found in the new distribution.
It represents only the items of direct commitment to give insight into commitments for REP-2005 repositories in the [ROS 2 Roadmap](https://docs.ros.org/en/rolling/Roadmap.html).
For a full list of important completed changes in the project, see the Migration Guides [Migration Guides][migration-guides].

## Lyrical Roadmap

- [ ] :roadmap-size-large: :roadmap-note-in-progress: - Controller Server and Plugin factor for centralized path handling and crosstrack error estimation and enforcement.
- [ ] :roadmap-size-large: :roadmap-note-in-progress: - Massive refactor of `rclcpp` types to `nav2` types for ROS API abstraction.
- [x] :roadmap-size-medium: - Improve dynamic feasibility on approach to goal on all control and behavior plugins.
- [x] :roadmap-size-medium: - Introduce Vector Object Server to augment Costmap Filters with polygons rather than annotated masks.
- [ ] :roadmap-size-medium: - [Continued Route Server][]
- [x] :roadmap-size-medium: - Add Pause and Resume feature to Nav2 behavior tree tasks.
- [x] :roadmap-size-medium: - Use PointCloud Transport for all pointcloud subscriptions.
- [ ] :roadmap-size-medium: - Update and refine behavior trees for more intelligent behavior.
- [x] :roadmap-size-medium: - Fix for statics in Smac Planner to allow for multiple instances in a server not to conflict.
- [ ] :roadmap-size-large: - Create initial prototype of a framework for environmental modeling, sensor processing, and external model integration.

[Continued Route Server]: https://github.com/ros-navigation/navigation2/issues/5082

## Kilted Roadmap

- [x] :roadmap-size-large: - Release of Route server.
- [x] :roadmap-size-medium: - Allow non-orientation sp. for smac planner goals.
- [x] :roadmap-size-medium: - Allow docking server to operate forward and backwards.

## Jazzy Roadmap

- [x] :roadmap-size-large: - [Smac Planner Improvements][]
- [x] :roadmap-size-small: - Get CI Green Again.
- [x] :roadmap-size-medium: - Various MPPI Improvements.
- [ ] :roadmap-size-medium: :roadmap-note-incomplete: - [Fuse Migration][]
- [x] :roadmap-size-medium: - [Ignition Migration][]
- [ ] :roadmap-size-large: :roadmap-note-in-progress: - [Route Graph Planner][]
- [x] :roadmap-size-medium: - Provide Advanced Capabilities Tutorials (e.g. gps, vio).
- [x] :roadmap-size-medium: - ROS Time Respect Across Stack.
- [x] :roadmap-size-medium: - TwistStamped Migration.
- [x] :roadmap-size-medium: - Velocity-Scheduled Collision Monitor Polygons.
- [x] :roadmap-size-medium: - Nav2 auto-docking capability.

[Smac Planner Improvements]: https://github.com/ros-navigation/navigation2/issues/3172
[Fuse Migration]: https://github.com/ros-navigation/navigation2/issues/2598
[Ignition Migration]: https://github.com/ros-navigation/navigation2/issues/2997
[Route Graph Planner]: https://github.com/ros-navigation/navigation2/issues/2229

## Iron Roadmap

- [x] :roadmap-size-medium: - [Pluginize Navigators][]
- [x] :roadmap-size-mega: - [MPPI Controller][]
- [ ] :roadmap-size-large: :roadmap-note-incomplete: - [Route Graph Planner][]
- [x] :roadmap-size-small: - 90% unit test coverage.
- [x] :roadmap-size-medium: - [Velocity Smoother][]
- [ ] :roadmap-size-medium: :roadmap-note-incomplete: - ROS Time Respect Across Stack.

[Pluginize Navigators]: https://github.com/ros-navigation/navigation2/issues/3335
[MPPI Controller]: https://github.com/ros-navigation/navigation2/pull/3350
[Route Graph Planner]: https://github.com/ros-navigation/navigation2/issues/2229
[Velocity Smoother]: https://github.com/ros-navigation/navigation2/pull/2964

## Humble Roadmap

- [x] :roadmap-size-medium: - [Nav2 1 Node Per Server][]
- [x] :roadmap-size-large: - [Smac Lattice Planner][]
- [x] :roadmap-size-medium: - [Safety Collision Nodes][]
- [x] :roadmap-size-small: - [Fix Min Range Bug][]
- [x] :roadmap-size-small: - [Move Development from Master to Rolling][]
- [x] :roadmap-size-medium: - Push Test Coverage to 88%.
- [x] :roadmap-size-medium: - [Complete First Time Guide][]
- [x] :roadmap-size-small: - [Rotation Shim Controller][]
- [x] :roadmap-size-medium: - [Dynamic Composition][]

[Nav2 1 Node Per Server]: https://github.com/ros-navigation/navigation2/issues/816
[Smac Lattice Planner]: https://github.com/ros-navigation/navigation2/issues/1710
[Safety Collision Nodes]: https://github.com/ros-navigation/navigation2/issues/1899
[Fix Min Range Bug]: https://github.com/ros-navigation/navigation2/pull/2460
[Move Development from Master to Rolling]: https://github.com/ros-navigation/navigation2/issues/2337
[Complete First Time Guide]: https://github.com/ros-navigation/navigation2/issues/1589
[Rotation Shim Controller]: https://github.com/ros-navigation/navigation2/pull/2718
[Dynamic Composition]: https://github.com/ros-navigation/navigation2/issues/2147
