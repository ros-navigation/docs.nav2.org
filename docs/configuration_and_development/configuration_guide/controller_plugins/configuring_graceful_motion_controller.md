# Graceful Controller { #graceful-controller }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_graceful_controller).

The graceful controller implements a controller based on the works of Jong Jin Park and Benjamin Kuipers in "A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment" (ICRA 2011). In this implementation, a *motion_target* is set at a distance away from the robot that is exponentially stable to generate a smooth trajectory for the robot to follow.

See the package's `README` for more complete information.

## Graceful Controller Parameters

### **`max_lookahead`**

Type: `double` Default: `1.0`

:   The maximum lookahead distance (m) to use when selecting a target pose for the underlying control law. Using poses that are further away will generally result in smoother operations, but simulating poses that are very far away can result in reduced performance, especially in tight or cluttered environments. If the controller cannot forward simulate to a pose this far away without colliding, it will iteratively select a target pose that is closer to the robot.

### **`min_lookahead`**

Type: `double` Default: `0.25`

:   The minimum lookahead distance (m) to use when selecting a target pose for the underlying control law. This parameter avoids instability when an unexpected obstacle appears in the path of the robot by returning failure, which typically triggers replanning.

### **`k_phi`**

Type: `double` Default: `2.0`

:   Ratio of the rate of change in phi to the rate of change in r. Controls the convergence of the slow subsystem. If this value is equal to zero, the controller will behave as a pure waypoint follower. A high value offers extreme scenario of pose-following where theta is reduced much faster than r. The referenced paper calls this *k1*.

### **`k_delta`**

Type: `double` Default: `1.0`

:   Constant factor applied to the heading error feedback. Controls the convergence of the fast subsystem. The bigger the value, the robot converge faster to the reference heading. The referenced paper calls this *k2*.

### **`beta`**

Type: `double` Default: `0.4`

:   Constant factor applied to the path curvature. This value must be positive. Determines how fast the velocity drops when the curvature increases.

### **`lambda`**

Type: `double` Default: `2.0`

:   Constant factor applied to the path curvature. This value must be greater or equal to `1`. Determines the sharpness of the curve: higher lambda implies sharper curves.

### **`v_linear_min`**

Type: `double` Default: `0.1`

:   Minimum linear velocity (m/s).

### **`v_linear_max`**

Type: `double` Default: `0.5`

:   Maximum linear velocity (m/s).

### **`v_angular_max`**

Type: `double` Default: `1.0`

:   Maximum angular velocity (rad/s) produced by the control law.

### **`v_angular_min_in_place`**

Type: `double` Default: `0.25`

:   Minimum angular velocity (rad/s) produced by the control law when rotating in place. This value should be based on the minimum rotation speed controllable by the robot.

### **`slowdown_radius`**

Type: `double` Default: `1.5`

:   Radius (m) around the goal pose in which the robot will start to slow down.

### **`deceleration_max`**

Type: `double` Default: `2.5`

:   Maximum deceleration (m/s²) used to compute a velocity limit based on distance to the goal.

### **`initial_rotation`**

Type: `bool` Default: `true`

:   Enable a rotation in place to the goal before starting the path. The control law may generate large sweeping arcs to the goal pose, depending on the initial robot orientation and `k_phi`, `k_delta`.

### **`initial_rotation_tolerance`**

Type: `double` Default: `0.75`

:   The difference in the path orientation and the starting robot orientation to trigger a rotate in place, if `initial_rotation` is enabled. This value is generally acceptable if continuous replanning is enabled. If not using continuous replanning, a lower value may perform better.

### **`prefer_final_rotation`**

Type: `bool` Default: `true`

:   The control law can generate large arcs when the goal orientation is not aligned with the path. If this is enabled, the orientation of the final pose will be ignored and the robot will follow the orientation of the path and will make a final rotation in place to the goal orientation.

### **`rotation_scaling_factor`**

Type: `double` Default: `0.5`

:   The scaling factor applied to the rotation in place velocity.

### **`allow_backward`**

Type: `bool` Default: `false`

:   Whether to allow the robot to move backward.

### **`in_place_collision_resolution`**

Type: `double` Default: `0.1`

:   When performing an in-place rotation after the XY goal tolerance has been met, this is the angle (in radians) between poses to check for collision.

### **`use_collision_detection`**

Type: `bool` Default: `true`

:   Whether to use collision detection to avoid obstacles.

### **`footprint_scaling_linear_vel`**

Type: `double` Default: `0.5`

:   The linear velocity threshold (m/s) below which footprint scaling is not applied. When the simulated velocity exceeds this value, the footprint is expanded proportionally to the ratio between the current velocity and `v_linear_max`. If the trajectory is in collision at the current speed, the controller retries with reduced speeds down to this threshold in steps of `footprint_scaling_step`.

### **`footprint_scaling_factor`**

Type: `double` Default: `0.25`

:   The maximum additional scaling factor applied to the robot footprint at `v_linear_max`. A value of `0.25` means the footprint can be expanded up to 125% of its original size at maximum velocity. The actual scaling increases linearly with velocity above `footprint_scaling_linear_vel`.

### **`footprint_scaling_step`**

Type: `double` Default: `0.1`

:   The step size (m/s) for reducing the simulated velocity when a trajectory is in collision at the current speed. The controller iterates from `v_linear_max` down to `footprint_scaling_linear_vel` in steps of this size, attempting to find a collision-free trajectory at a lower speed with a smaller footprint.

### **`obstacle_cost_margin`**

Type: `int` Default: `1`

:   The cost margin below the maximum valid cost that is considered "close to obstacles." When the maximum cost along the final approach trajectory is within this margin of the maximum valid cost, the controller searches for a safer alternative approach angle using spiral curves. A higher value triggers the search more aggressively. Must be less than the maximum non-obstacle cost value.

### **`final_rotation_search_step`**

Type: `double` Default: `0.1`

:   The angular step size (rad) used when searching for an alternative final approach angle that avoids obstacles. The controller sweeps through orientations in steps of this size to find a spiral curve approach that keeps the trajectory farther from obstacles using `obstacle_cost_margin`. Smaller values provide finer search granularity at the cost of increased computation.

### **`allow_parameter_qos_overrides`**

Type: `bool` Default: `true`

:   Whether to allow QoS profiles to be overwritten with parameterized values.

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
      plugin: nav2_graceful_controller::GracefulController
      min_lookahead: 0.25
      max_lookahead: 1.0
      initial_rotation: true
      initial_rotation_threshold: 0.75
      prefer_final_rotation: true
      allow_backward: false
      k_phi: 2.0
      k_delta: 1.0
      beta: 0.4
      lambda: 2.0
      v_linear_min: 0.1
      v_linear_max: 0.5
      v_angular_max: 5.0
      v_angular_min_in_place: 0.25
      slowdown_radius: 1.5
      deceleration_max: 2.5
      footprint_scaling_linear_vel: 0.5
      footprint_scaling_factor: 0.25
      footprint_scaling_step: 0.1
      obstacle_cost_margin: 1
      final_rotation_search_step: 0.1
```
