# Roadmaps

This is the list of major issues and features the Nav2 maintainers are committing for completion for various ROS 2 releases.
This is *not* an exhaustive list of planned features or what changes may be found in the new distribution.
It represents only the items of direct commitment to give insight into commitments for REP-2005 repositories in the [ROS 2 Roadmap](https://docs.ros.org/en/rolling/Roadmap.html).
For a full list of important completed changes in the project, see the Migration Guides [Migration Guides](../migration/index.md#migration-guides).

## Lyrical Roadmap

| Description                                                                                                                       | Size (Status)         |
|-----------------------------------------------------------------------------------------------------------------------------------|-----------------------|
| Controller Server and Plugin<br/>factor for centralized path<br/>handling and crosstrack error<br/>estimation and enforcement.    | Large (In Progress)   |
| Massive refactor of `rclcpp`<br/>types to `nav2` types for<br/>ROS API abstraction                                                | Large (in progress)   |
| Improve dynamic feasibility<br/>on approach to goal on all<br/>control and behavior plugins                                       | Medium (DONE)         |
| Introduce Vector Object Server<br/>to augment Costmap Filters<br/>with polygons rather than<br/>annotated masks                   | Medium  (DONE)        |
| [Continued Route Server][]                                                                                                        | Medium                |
| Add Pause and Resume feature<br/>to Nav2 behavior tree tasks                                                                      | Medium (DONE)         |
| Use PointCloud Transport for<br/>all pointcloud subscriptions                                                                     | Medium (DONE)         |
| Update and refine behavior<br/>trees for more intelligent<br/>behavior                                                            | Medium                |
| Fix for statics in Smac<br/>Planner to allow for multiple<br/>instances in a server not to<br/>conflict                           | Medium (DONE)         |
| Create initial prototype of<br/>a framework for environmental<br/>modeling, sensor processing,<br/>and external model integration | Large                 |

[Continued Route Server]: https://github.com/ros-navigation/navigation2/issues/5082

## Kilted Roadmap

| Description                                               | Size (Status)  |
|-----------------------------------------------------------|----------------|
| Release of Route server                                   | Large (DONE)   |
| Allow non-orientation sp. for<br/>smac planner goals      | Medium (DONE)  |
| Allow docking server to operate<br/>forward and backwards | Medium (DONE)  |

## Jazzy Roadmap

| Description                                                 | Size (Status)        |
|-------------------------------------------------------------|----------------------|
| [Smac Planner Improvements][]                               | Large (DONE)         |
| Get CI Green Again                                          | Small (DONE)         |
| Various MPPI Improvements                                   | Medium (DONE)        |
| [Fuse Migration][]                                          | Medium  (incomplete) |
| [Ignition Migration][]                                      | Medium (DONE)        |
| [Route Graph Planner][]                                     | Large (in progress)  |
| Provide Advanced Capabilities<br/>Tutorials (e.g. gps, vio) | Medium (DONE)        |
| ROS Time Respect Across Stack                               | Medium (DONE)        |
| TwistStamped Migration                                      | Medium (DONE)        |
| Velocity-Scheduled Collision<br/>Monitor Polygons           | Medium (DONE)        |
| Nav2 auto-docking capability                                | Medium (DONE)        |

[Smac Planner Improvements]: https://github.com/ros-navigation/navigation2/issues/3172
[Fuse Migration]: https://github.com/ros-navigation/navigation2/issues/2598
[Ignition Migration]: https://github.com/ros-navigation/navigation2/issues/2997
[Route Graph Planner]: https://github.com/ros-navigation/navigation2/issues/2229

## Iron Roadmap

| Plugin Name                   | Size (Status)       |
|-------------------------------|---------------------|
| [Pluginize Navigators][]      | Medium  (DONE)      |
| [MPPI Controller][]           | Very Large (DONE)   |
| [Route Graph Planner][]       | Large (incomplete)  |
| 90% unit test coverage        | Small  (DONE)       |
| [Velocity Smoother][]         | Medium (DONE)       |
| ROS Time Respect Across Stack | Medium (incomplete) |

[Pluginize Navigators]: https://github.com/ros-navigation/navigation2/issues/3335
[MPPI Controller]: https://github.com/ros-navigation/navigation2/pull/3350
[Route Graph Planner]: https://github.com/ros-navigation/navigation2/issues/2229
[Velocity Smoother]: https://github.com/ros-navigation/navigation2/pull/2964

## Humble Roadmap

| Plugin Name                                     | Size (Status)  |
|-------------------------------------------------|----------------|
| [Nav2 1 Node Per Server][]                      | Medium  (DONE) |
| [Smac Lattice Planner][]                        | Large (DONE)   |
| [Safety Collision Nodes][]                      | Medium (DONE)  |
| [Fix Min Range Bug][]                           | Small  (DONE)  |
| [Move Development from Master to Rolling][]     | Small (DONE)   |
| Push Test Coverage to 88%                       | Medium (DONE)  |
| [Complete First Time Guide][]                   | Medium (DONE)  |
| [Rotation Shim Controller][]                    | Small (DONE)   |
| [Dynamic Composition][]                         | Medium (DONE)  |

[Nav2 1 Node Per Server]: https://github.com/ros-navigation/navigation2/issues/816
[Smac Lattice Planner]: https://github.com/ros-navigation/navigation2/issues/1710
[Safety Collision Nodes]: https://github.com/ros-navigation/navigation2/issues/1899
[Fix Min Range Bug]: https://github.com/ros-navigation/navigation2/pull/2460
[Move Development from Master to Rolling]: https://github.com/ros-navigation/navigation2/issues/2337
[Complete First Time Guide]: https://github.com/ros-navigation/navigation2/issues/1589
[Rotation Shim Controller]: https://github.com/ros-navigation/navigation2/pull/2718
[Dynamic Composition]: https://github.com/ros-navigation/navigation2/issues/2147
