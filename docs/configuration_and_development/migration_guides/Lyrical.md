# Lyrical to M-Turtle { #lyrical-to-m-turtle }

## Transform Staleness Checking and Time-Coherent Transformations

The navigation stack can behave incorrectly when some of its input transformations are not provided for a long period of time, possibly leading to loss of control and collisions. It is advised to configure the stack to fail in this condition.

The Following Nodes can now optionally detect stale transforms:

- Controller Server

A new transform_staleness_threshold parameter specifies the maximum allowed age of this transform in seconds. Positive values enable the staleness check, while values less than or equal to 0.0 disable it. The default value is 0.0, so no configuration changes are required to retain the previous behavior with respect to stale-transform rejection.

This parameter should be configured to a value bigger than the maximum periodiciy of the non static transformations in your tree. A couple of seconds is usually a reasonable conservative choice, but a stricter check is advised for systems where timely updates are critical.

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
