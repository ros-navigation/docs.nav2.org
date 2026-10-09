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

### Single Lifecycle Manager in Nav2 Bringup

[PR #6410](https://github.com/ros-navigation/navigation2/pull/6410) combines the separate lifecycle managers for localization, navigation, SLAM, keepout zones, speed zones and the loopback simulator into a single `lifecycle_manager_nav2`, started by `bringup_launch.py`. Measured on the Turtlebot 4 simulation launch, which previously ran four lifecycle managers, this reduces Nav2's CPU use by around 25%. The nested launch files in `nav2_bringup` no longer start a lifecycle manager of their own. Each has a `get_lifecycle_nodes(context)` function that returns its lifecycle node names, which `bringup_launch.py` collects into the `node_names` of the single manager.

Launching a nested file by itself, such as `ros2 launch nav2_bringup navigation_launch.py`, now leaves the servers unconfigured. Launch `bringup_launch.py` instead and use its arguments to turn off what you do not need, for example `use_localization:=False` when SLAM or another source provides the map and the `map` to `odom` transform. Anything that used the old manager names, such as `lifecycle_manager_navigation/manage_nodes`, should use `lifecycle_manager_nav2` instead; the Nav2 RViz panel is already updated.

Move your own nested launch files to the same pattern: remove the lifecycle manager from each one, add a `get_lifecycle_nodes(context)` function that returns its lifecycle node names, and build the `node_names` of your single manager from those functions plus your own nodes. The [task server tutorial][adding-a-new-nav2-task-server] shows this.

If you would rather run a launch file on its own, start a lifecycle manager beside it.

## Changes in message types

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
