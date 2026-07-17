# Rotation Shim Controller { #rotation-shim-controller }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/main/nav2_rotation_shim_controller).

The `nav2_rotation_shim_controller` will check the rough heading difference with respect to the robot and a newly received path. If within a threshold, it will pass the request onto the `primary_controller` to execute the task. If it is outside of the threshold, this controller will rotate the robot in place towards that path heading. Once it is within the tolerance, it will then pass off control-execution from this rotation shim controller onto the primary controller plugin. At this point, the robot's main plugin will take control for a smooth hand off into the task.

When the `rotate_to_goal_heading` parameter is set to true, this controller is also able to take back control of the robot when reaching the XY goal tolerance of the goal checker. In this case, the robot will rotate towards the goal heading until the goal checker validate the goal and ends the current navigation task.

The `RotationShimController` is most suitable for:

- Robots that can rotate in place, such as differential and omnidirectional robots.
- Preference to rotate in place when starting to track a new path that is at a significantly different heading than the robot's current heading – or when tuning your controller for its task makes tight rotations difficult.
- Using planners that are non-kinematically feasible, such as NavFn, Theta\*, or Smac 2D (Feasible planners such as Smac Hybrid-A\* and State Lattice will start search from the robot's actual starting heading, requiring no rotation since their paths are guaranteed drivable by physical constraints).

See the package's `README` for more complete information.

<div class="video-container">
  <iframe width="708" height="400" src="https://www.youtube.com/embed/t-g2CBGByEw?playlist=t-g2CBGByEw&autoplay=1&mute=1&loop=1" frameborder="1" allowfullscreen></iframe>
</div>

## Rotation Shim Controller Parameters

### **`angular_dist_threshold`**

Type: `double` Default: `0.785`

:   Maximum angular distance, in radians, away from the path heading to trigger rotation

### **`angular_disengage_threshold`**

Type: `double` Default: `0.3925`

:   The threshold to the path's heading before disengagement (radians). This allows for better alignment before passing to the child controller when engaged.

### **`forward_sampling_distance`**

Type: `double` Default: `0.5`

:   Forward distance, in meters, along path to select a sampling point to use to approximate path heading. This distance should not be larger than the path handler's prune distance.

### **`rotate_to_heading_angular_vel`**

Type: `double` Default: `1.8`

:   Angular rotational velocity, in rad/s, to rotate to the path heading

### **`primary_controller`**

Type: `string` Default: `N/A`

:   Internal controller plugin to use for actual control behavior after rotating to heading

### **`max_angular_accel`**

Type: `double` Default: `3.2`

:   Maximum angular acceleration for rotation to heading (rad/s/s)

### **`max_cost_threshold`**

Type: `double` Default: `254.0`

:   Maximum footprint cost threshold to detect a collision. Defaults to 254.0 i.e., LETHAL_OBSTACLE. If needed, it can be adjusted to match the behavior of the primary controller plugin.

### **`simulate_ahead_time`**

Type: `double` Default: `1.0`

:   Time in seconds to forward simulate a rotation command to check for collisions. If a collision is found, forwards control back to the primary controller plugin.

### **`rotate_to_goal_heading`**

Type: `bool` Default: `false`

:   If true, the rotationShimController will take back control of the robot when in XY tolerance of the goal and start rotating towards the goal heading.

### **`rotate_to_heading_once`**

Type: `bool` Default: `false`

:   If true, the rotationShimController will only rotate to heading once on a new goal, not each time a path is set.

### **`closed_loop`**

Type: `bool` Default: `true`

:   If false, the rotationShimController will use the last commanded velocity as the next iteration's current velocity. When acceleration limits are set appropriately and the robot's controllers are responsive, this can be a good assumption. If true, it will use odometry to estimate the robot's current speed. In this case it is important that the source is high-rate and low-latency to account for control delay.

### **`use_path_orientations`**

Type: `bool` Default: `false`

:   If true, the controller will use the orientations of the path points to compute the heading of the path instead of computing the heading from the path point's relative locations. If false, the controller will compute the heading from the path point's relative locations instead of using the path point orientations. Use for feasible planners like the Smac Planner which generate feasible paths with orientations for forward and reverse motion.

## Example

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      angular_dist_threshold: 0.785
      forward_sampling_distance: 0.5
      angular_disengage_threshold: 0.3925
      rotate_to_heading_angular_vel: 1.8
      max_angular_accel: 3.2
      max_cost_threshold: 254.0
      simulate_ahead_time: 1.0
      rotate_to_goal_heading: false

      # Primary controller params can be placed here below
      primary_controller:
        plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        # ...
```
