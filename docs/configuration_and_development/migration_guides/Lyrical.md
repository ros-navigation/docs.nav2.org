# Lyrical to M-Turtle { #lyrical-to-m-turtle }

## Migration Actions and Deprecations

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

## New Features and Improvements

### Static Layer Overlays Without Resizing the Master

[PR #6488](https://github.com/ros-navigation/navigation2/pull/6488) adds the Static Layer parameter `resize_master`, which defaults to `true`. Existing configurations require no changes: a Static Layer still resizes a non-rolling master costmap to match its incoming map.

Set `resize_master: false` for additional Static Layers whose maps should not control the master's geometry. For example, a Vector Object Server can publish an obstacle map with a smaller or changing extent while the primary Static Layer continues to own the site map geometry.

### Assisted Teleop Times Out on Stale Teleop Commands

The `AssistedTeleop` behavior now stops the robot and fails the action with the new `TELEOP_INPUT_TIMEOUT` error code (733) when no teleop command has been received within the new `teleop_command_timeout` parameter (default `0.25` s).

### Transform Staleness Checking and Time-Coherent Transformations

The navigation stack can behave incorrectly when some of its input transformations are not provided for a long period of time, possibly leading to loss of control and collisions. It is advised to configure the stack to fail in this condition.

The following components can detect stale transforms when their staleness check is enabled:

- Controller Server and its path handler
- Local and global costmaps, their clearing services, Static Layer, and Asymmetric Inflation Layer
- Keepout, Speed, Binary, and Zone Parameter costmap filters
- Spin, Drive On Heading, Back Up, and Assisted Teleop behaviors
- Docking Server and the Simple Charging and Simple Non-Charging Dock plugins
- Following Server
- Collision Monitor and Collision Detector, including collision sources, polygons, and exclusion zones
- BT Navigator (NavigateToPose and NavigateThroughPoses) and GetCurrentPose, RemovePassedGoals, TruncatePathLocal, GoalReached, IsGoalNearby, ArePosesNear, DistanceTraveled, and DistanceController BT nodes
- Planner Server (through its costmap), Route Server (RouteTracker and GoalIntentExtractor)
- Vector Object Server
- Footprint transform helpers

The `transform_staleness_threshold` parameter specifies the maximum allowed age of a latest transform in seconds. Positive values enable the staleness check, while values less than or equal to 0.0 disable it. The default is `0.0` for most components. Collision Monitor and Collision Detector default to `1.0`, enabling the check.

When the check is enabled and the transform is older than the configured threshold, the affected component reports an error or rejects the transform.

This parameter should be configured to a value bigger than the maximum period of the dynamic transforms in your tree. A couple of seconds is usually a reasonable conservative choice, but a stricter check is advised for systems where timely updates are critical.

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
