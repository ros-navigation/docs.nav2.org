# Lyrical to M-Turtle { #lyrical-to-m-turtle }

## Migration Actions and Deprecations

### Transform Manager API

[PR #6651](https://github.com/ros-navigation/navigation2/pull/6651) introduces `nav2::TransformManager` to simplify and standardize how Nav2 receives, looks up, and broadcasts TF transforms. Its common lookup methods make the choice between the latest available transform and a transform at a message's timestamp explicit. This reduces repeated TF2 setup and helps components apply the same timing and staleness rules.

Custom servers and plugins should create the manager with `nav2::create_transform_manager(node)` from `nav2_ros_common/tf_manager.hpp`. Use it for lookups and broadcasting in place of direct TF2 calls, the old `nav2::TransformBuffer` alias and listener factories, and the removed transform helpers in `nav2_util/robot_utils.hpp`.

| Use the new API when you need... | New `nav2::TransformManager` API | Previous API |
| --- | --- | --- |
| The latest robot or other frame pose, with an optional age limit | `getLatestPose(target_frame, source_frame, staleness_threshold)` | `nav2_util::getCurrentPose()` or a latest TF lookup followed by pose conversion |
| The latest available transform between frames, with an optional age limit | `getLatestTransform(target_frame, source_frame, staleness_threshold)` | `nav2_util::getTransform()` or `lookupTransform()` with time zero |
| A transform at a sensor or computed result's timestamp | `getStampedTransform(target_frame, source_frame, stamp, transform_tolerance)` | `nav2_util::getTransform()` or `lookupTransform()` at a specified time |
| A transform between two timestamps through a fixed frame | `getStampedTransform(target_frame, target_time, source_frame, source_time, fixed_frame, transform_tolerance)` | The corresponding advanced `lookupTransform()` overload |
| A stamped pose expressed in another frame at the pose's timestamp | `transformStampedPose(target_frame, input_pose, transform_tolerance)` | `nav2_util::transformPoseInTargetFrame()` or `transform()` |
| Publishing dynamic or static transforms | `sendTransform(transform)` or `sendStaticTransform(transform)` | A separate `tf2_ros::TransformBroadcaster` or `tf2_ros::StaticTransformBroadcaster` |

Latest lookups use the latest common time in the TF chain and do not wait for unavailable TF. They can reject stale dynamic transforms when a positive `staleness_threshold` is supplied. Stamped lookups use the requested time and can wait up to `transform_tolerance` (`tf2::Duration`) for the required TF; they do not apply a current-time staleness check. A zero stamp requests the latest common time without an age check. See the [Transform Synchronization Strategy][transform-synchronization-strategy] for guidance on choosing between them.

Lookup methods return `std::optional`; check the result before using it. They log failures by default. Pass `nav2::Log::Disabled` as the final argument for expected failures or when the caller handles diagnostics. For points, point clouds, and other message types, obtain the appropriate transform from the manager and apply it with `tf2::doTransform`.

The manager owns its TF buffer and listener. The listener starts by default; use `startListener()` and `stopListener()` for lifecycle transitions. For native TF integrations that require a `tf2_ros::Buffer`, such as `laser_geometry`, use `getRawBuffer()`. Pass the manager to `nav2::create_message_filter()` for TF-aware subscriptions.

### Behavior Server plugin parameters are now namespaced

The behavior plugins' parameters are now declared under the plugin's name from `behavior_plugins`, in the same way the `acceleration_limit`, `deceleration_limit` and `minimum_speed` parameters of `BackUp` and `DriveOnHeading` already were. The old node-level names are no longer read, so existing configurations must be updated or the defaults will silently apply:

| Old (node-level)       | New (per plugin)                                                     |
|------------------------|----------------------------------------------------------------------|
| `simulate_ahead_time`  | `spin.simulate_ahead_time`, `backup.simulate_ahead_time`, `drive_on_heading.simulate_ahead_time` |
| `max_rotational_vel`   | `spin.max_rotational_vel`                                            |
| `min_rotational_vel`   | `spin.min_rotational_vel`                                            |
| `rotational_acc_lim`   | `spin.rotational_acc_lim`                                            |
| `projection_time`      | `assisted_teleop.projection_time`                                    |
| `simulation_time_step` | `assisted_teleop.simulation_time_step`                               |
| `cmd_vel_teleop`       | `assisted_teleop.cmd_vel_teleop`                                     |

Substitute your own plugin names if they differ from the defaults. Simply move the parameter under the plugin's declaration in your configuration.

### Single Lifecycle Manager in Nav2 Bringup

[PR #6410](https://github.com/ros-navigation/navigation2/pull/6410) combines the separate lifecycle managers for localization, navigation, SLAM, keepout zones, speed zones and the loopback simulator into a single `lifecycle_manager_nav2`, started by `bringup_launch.py`. Measured on the Turtlebot 4 simulation launch, which previously ran four lifecycle managers, this reduces Nav2's CPU use by around 25%. The nested launch files in `nav2_bringup` no longer start a lifecycle manager of their own. Each has a `get_lifecycle_nodes(context)` function that returns its lifecycle node names, which `bringup_launch.py` collects into the `node_names` of the single manager.

Launching a nested file by itself, such as `ros2 launch nav2_bringup navigation_launch.py`, now leaves the servers unconfigured. Launch `bringup_launch.py` instead and use its arguments to turn off what you do not need, for example `use_localization:=False` when SLAM or another source provides the map and the `map` to `odom` transform. Anything that used the old manager names, such as `lifecycle_manager_navigation/manage_nodes`, should use `lifecycle_manager_nav2` instead; the Nav2 RViz panel is already updated.

Move your own nested launch files to the same pattern: remove the lifecycle manager from each one, add a `get_lifecycle_nodes(context)` function that returns its lifecycle node names, and build the `node_names` of your single manager from those functions plus your own nodes. The [task server tutorial][adding-a-new-nav2-task-server] shows this.

If you would rather run a launch file on its own, start a lifecycle manager beside it.

## Default BT Updates

### Backup recovery now waits for the robot to move

[PR #6635](https://github.com/ros-navigation/navigation2/pull/6635) replaces `BackUp` with the `BackupWithReset` subtree in six bundled navigation behavior trees with recovery actions. After a successful backup, the subtree skips another backup near the same pose. It allows backup again once the robot has moved at least `reset_distance` from the last backup pose. The backup history resets when the behavior tree is reloaded.

The updated trees also set `wrap_around="true"` on their `RecoveryActions` `RoundRobin` node so the recovery sequence can cycle (and for example spin can be executed multiple times). If you maintain a copy of one of these trees and want the same behavior, replace its recovery `BackUp` node with the `BackupWithReset` subtree and set `wrap_around="true"` on the `RoundRobin`. Adjust the frame names and reset distance for your robot.

## Changes in FollowObject Action

FollowObject.action error codes are changed as below. If you are using this action you will have to adapt to handle new error codes:

| Old                             | New                             |
| ------------------------------- | ------------------------------- |
| `TF_ERROR = 901`                | `TF_ERROR = 801`                |
| `FAILED_TO_DETECT_OBJECT = 902` | `FAILED_TO_DETECT_OBJECT = 802` |
| `FAILED_TO_CONTROL = 903`       | `FAILED_TO_CONTROL = 803`       |
| `TIMEOUT = 904`                 | `TIMEOUT = 804`                 |
| `UNKNOWN = 999`                 | `UNKNOWN = 899`                 |

## New Features and Improvements

### Static Layer Overlays Without Resizing the Master

[PR #6488](https://github.com/ros-navigation/navigation2/pull/6488) adds the Static Layer parameter `resize_master`, which defaults to `true`. Existing configurations require no changes: a Static Layer still resizes a non-rolling master costmap to match its incoming map.

Set `resize_master: false` for additional Static Layers whose maps should not control the master's geometry. For example, a Vector Object Server can publish an obstacle map with a smaller or changing extent while the primary Static Layer continues to own the site map geometry.

### Assisted Teleop Times Out on Stale Teleop Commands

The `AssistedTeleop` behavior now stops the robot and fails the action with the new `TELEOP_INPUT_TIMEOUT` error code (733) when no teleop command has been received within the new `teleop_command_timeout` parameter (default `0.25` s).

### Include Specific rclcpp Headers

[PR #6558](https://github.com/ros-navigation/navigation2/pull/6558) replaces the broad `rclcpp/rclcpp.hpp` includes throughout Nav2 with the specific headers each file needs to speed up compilation and reduce build memory usage. We now recommend including specific `rclcpp` headers instead of globally including `rclcpp/rclcpp.hpp`.

See the [ROS 2 documentation note on the `rclcpp` convenience header](https://github.com/ros2/ros2_documentation/blob/rolling/source/_internal/Rclcpp-Convenience-Header-Note.rst) for more details.

### MPPI Critics for Omnidirectional Velocity Limits

[PR #6498](https://github.com/ros-navigation/navigation2/pull/6498) adds two MPPI critics. The ConstraintCritic bounds `vx` and `vy` separately. This creates a rectangular velocity space, with maximum translational velocity being `sqrt(vx_max² + vy_max²)`. Therefore, any critic rewarding speed on an omnidirectional platform will lead to the robot driving diagonally, as `sqrt(vx_max² + vy_max²)` is larger than `vx_max` and `vy_max` alone.

- The [OmniConstraintCritic][mppi-omni-constraint-critic] (`mppi::critics::OmniConstraintCritic`) is intended to be used for omnidirectional models instead of the ConstraintCritic. Its velocity space is an ellipse spanned by the controller's `vx_max`, `vx_min` and `vy_max`, so it keeps the same functionality as the ConstraintCritic, while preventing speed rewarding critics from forcing the robot to drive diagonally.
- The [MecanumCritic][mppi-mecanum-critic] (`mppi::critics::MecanumCritic`) is intended to be used to constrain a mecanum robot's drivetrain to its reachable velocity. This, because a mecanum model's velocities `vx`, `vy` and `wz` are not independent from each other. Driving with `vx_max` is only possible when `vy` and `wz` are zero, as otherwise a wheel would saturate.

Neither critic is enabled by default; existing configurations are unaffected.

### Transform Staleness Checking

When a dynamic TF publisher stops updating, its last transform can still be returned as the latest available one. Navigation based on that old pose can produce incorrect commands or miss hazards. Staleness checking lets a component reject an outdated latest transform and report the failure so the robot can respond safely.

The `transform_staleness_threshold` parameter sets the maximum allowed age in seconds for a latest transform. A positive value enables the check; `0.0` or a negative value disables it. Most components default to `0.0`, while Collision Monitor and Collision Detector default to `1.0`. Static-only TF chains are exempt. Choose a threshold that covers the normal update period and expected latency of your dynamic transforms, but still catches a stopped publisher promptly.

The check is available across applicable Nav2 servers, plugins, costmap layers, filters, and BT nodes through the related [staleness changes](https://github.com/ros-navigation/navigation2/pull/6391), [TF lookup changes](https://github.com/ros-navigation/navigation2/pull/6436), and [TF manager PR #6651](https://github.com/ros-navigation/navigation2/pull/6651). Set the parameter in the relevant component's configuration when you want it to reject stale TF. Stamped lookups use the data's timestamp and do not use this age check.

Collision Monitor and Collision Detector exclusion zones no longer use `<zone name>.frame_hold_timeout`. Configure `<zone name>.transform_staleness_threshold` instead to limit the age of a zone-frame transform; it inherits the node-level threshold when omitted.

The Behavior Server no longer reads `transform_tolerance`; remove it from Behavior Server configuration when migrating.

### ProgressChecker API Uses a Const Robot Pose

The `nav2_core::ProgressChecker::check()` interface now takes the current robot pose by const reference.

Previously:

```cpp
virtual bool check(
  geometry_msgs::msg::PoseStamped & current_pose) = 0;
```

Now:

```cpp
virtual bool check(
  const geometry_msgs::msg::PoseStamped & current_pose) = 0;
```
