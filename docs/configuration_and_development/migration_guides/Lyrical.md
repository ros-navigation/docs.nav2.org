# Lyrical to M-Turtle { #lyrical-to-m-turtle }

## Transform Staleness Checking and Time-Coherent Transformations

The Following Nodes can now optionally detect stale transforms used to obtain the robot pose in the local costmap's global frame:

- Controller Server

A new transform_staleness_threshold parameter specifies the maximum allowed age of this transform in seconds. Positive values enable the staleness check, while values less than or equal to 0.0 disable it. The default value is 0.0, so no configuration changes are required to retain the previous behavior with respect to stale-transform rejection.

When the check is enabled and the transform is older than the configured threshold, the Nodes will report an error.

Whenever possible the nodes will retrieve the required transformations once at the beginning of each execution cycle and reuses them / their timestamp across the rest of the computation. This ensures that components within an execution cycle operate using time-coherent information. Custom plugins performing related TF operations should use the timestamp supplied with these transformations / poses where appropriate, rather than independently requesting additional transformations.

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