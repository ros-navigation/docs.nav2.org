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

### Controller Server Transform Staleness Checking and Time-Coherent Robot Pose

The Controller Server can now optionally detect stale transforms used to obtain the robot pose in the local costmap's global frame.

A new transform_staleness_threshold parameter specifies the maximum allowed age of this transform in seconds. Positive values enable the staleness check, while values less than or equal to 0.0 disable it. The default value is 0.0, so no configuration changes are required to retain the previous behavior with respect to stale-transform rejection.

When the check is enabled and the transform is older than the configured threshold, the Controller Server reports a transform error rather than computing a velocity command.

The Controller Server also now retrieves the robot pose once at the beginning of each control cycle and reuses that pose and its timestamp for plan transformation, goal and progress checking, controller computation, and related feedback. This ensures that components within a control cycle operate using a time-coherent robot pose. Custom controller implementations performing related TF operations should use the timestamp supplied with the robot pose where appropriate, rather than independently requesting the latest transform, to remain time-coherent with the Controller Server.

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

Custom `ProgressChecker` plugins must update both the declaration and implementation of `check()` to use a `const geometry_msgs::msg::PoseStamped &`. No other behavioral changes are required for progress checker plugins as a result of this API change.
