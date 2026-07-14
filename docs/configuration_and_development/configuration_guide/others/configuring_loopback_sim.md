# Loopback Simulator { #loopback-simulator }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/main/nav2_loopback_sim).

The `nav2_loopback_sim` is a stand-alone simulator to create a "loopback" for non-physical simulation to replace robot hardware, physics simulators (Gazebo, Bullet, Isaac Sim, etc).
It computes the robot's odometry based on the command velocity's output request to create a perfect 'frictionless plane'-style simulation for unit testing, system testing, R&D on higher level systems, testing behaviors without concerning yourself with localization accuracy or system dynamics, and multirobot simulations.

## Parameters

### **`update_duration`**

Type: `double` Default: `0.01`

:   The duration between updates (s)

### **`base_frame_id`**

Type: `string` Default: `"base_footprint"`

:   The base frame to use.

### **`odom_frame_id`**

Type: `string` Default: `"odom"`

:   The odom frame to use.

### **`map_frame_id`**

Type: `string` Default: `"map"`

:   The map frame to use.

### **`scan_frame_id`**

Type: `string` Default: `"base_scan"`

:   The scan frame to use to publish a scan

### **`enable_stamped_cmd_vel`**

Type: `string` Default: `True`

:   Whether cmd_vel is stamped or unstamped (i.e. Twist or TwistStamped).

### **`scan_publish_dur`**

Type: `double` Default: `0.1`

:   The duration between publishing scan (in sec)

### **`odom_publish_dur`**

Type: `double` Default: `<update_duration>`

:   The duration between publishing odometry (in sec). Defaults to `update_duration`.

### **`publish_map_odom_tf`**

Type: `bool` Default: `true`

:   Whether or not to publish tf from `map_frame_id` to `odom_frame_id`

### **`publish_scan`**

Type: `bool` Default: `true`

:   Whether or not to publish a fake laser scan.

### **`publish_clock`**

Type: `bool` Default: `true`

:   Whether or not to publish simulated clock to `/clock`

### **`speed_factor`**

Type: `double` Default: `1.0`

:   Speed factor for the simulated clock, e.g. `2.0` runs simulation at 2x wall time.
    Only used when `publish_clock` is `true`. Dynamically reconfigurable.

### **`scan_range_min`**

Type: `double` Default: `0.05`

:   Minimum measurable distance from the scan in meters. Values below this are considered invalid.

### **`scan_range_max`**

Type: `double` Default: `30.0`

:   Maximum measurable distance from the scan in meters. Values beyond this are out of range.

### **`scan_angle_min`**

Type: `double` Default: `-3.14`

:   Starting angle of the scan in radians (leftmost angle)

### **`scan_angle_max`**

Type: `double` Default: `3.14`

:   Ending angle of the scan in radians (rightmost angle)

### **`scan_angle_increment`**

Type: `double` Default: `0.0261`

:   Angular resolution of the scan in radians (angle between consecutive measurements)

### **`scan_use_inf`**

Type: `bool` Default: `true`

:   Whether to use `inf` for out-of-range values.
    If `false`, values are set to `scan_range_max - 0.1` instead.

### **`scan_noise_std`**
Type: `double` Default: `0.01`

:   Standard deviation of Gaussian noise added to scan ranges (in meters).


## Example

```yaml
loopback_simulator:
  ros__parameters:
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    map_frame_id: "map"
    scan_frame_id: "base_scan"  # tb4_loopback_simulator.launch.py remaps to 'rplidar_link'
    update_duration: 0.02
    publish_scan: true
    publish_clock: true
    speed_factor: 1.0
    scan_range_min: 0.05
    scan_range_max: 30.0
    scan_angle_min: -3.1415
    scan_angle_max: 3.1415
    scan_angle_increment: 0.02617
    scan_use_inf: true
    scan_noise_std: 0.01
```
