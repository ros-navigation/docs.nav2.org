# Lyrical to M-Turtle { #lyrical-to-m-turtle }

## Controller Server Transform Staleness Checking and Time-Coherent Robot Pose

The Controller Server can now optionally detect stale transforms used to obtain the robot pose in the local costmap's global frame.

A new transform_staleness_threshold parameter specifies the maximum allowed age of this transform in seconds. Positive values enable the staleness check, while values less than or equal to 0.0 disable it. The default value is 0.0, so no configuration changes are required to retain the previous behavior with respect to stale-transform rejection.

When the check is enabled and the transform is older than the configured threshold, the Controller Server reports a transform error rather than computing a velocity command.

The Controller Server also now retrieves the robot pose once at the beginning of each control cycle and reuses that pose and its timestamp for plan transformation, goal and progress checking, controller computation, and related feedback. This ensures that components within a control cycle operate using a time-coherent robot pose. Custom controller implementations performing related TF operations should use the timestamp supplied with the robot pose where appropriate, rather than independently requesting the latest transform, to remain time-coherent with the Controller Server.

## ProgressChecker API Uses a Const Robot Pose

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