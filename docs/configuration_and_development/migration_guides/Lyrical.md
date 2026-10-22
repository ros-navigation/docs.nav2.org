# Lyrical to M-Turtle { #lyrical-to-m-turtle }

## Migration Actions and Deprecations

## New Features and Improvements

### Static Layer Overlays Without Resizing the Master

[PR #6488](https://github.com/ros-navigation/navigation2/pull/6488) adds the Static Layer parameter `resize_master`, which defaults to `true`. Existing configurations require no changes: a Static Layer still resizes a non-rolling master costmap to match its incoming map.

Set `resize_master: false` for additional Static Layers whose maps should not control the master's geometry. For example, a Vector Object Server can publish an obstacle map with a smaller or changing extent while the primary Static Layer continues to own the site map geometry.
<<<<<<< HEAD
=======

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
>>>>>>> 64a93c2 (docs(mppi): add information about OmniConstraintCritic and MecanumCritic (#993))
