# Costmap 2D { #costmap-2d }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d).

The Costmap 2D package implements a 2D grid-based costmap for environmental representations and a number of sensor processing plugins (AI outputs, depth sensor obstacle buffering, semantic information, etc).
It is used in the planner and controller servers for creating the space to check for collisions or higher cost areas to negotiate around.

## Costmap2D ROS Parameters

### **`always_send_full_costmap`**

Type: `bool` Default: `false`

:   Whether to send the full costmap on every update instead of only incremental updates.

### **`introspection_mode`**

Type: `string` Default: `"disabled"`

:   The introspection mode for services and actions. Options are `"disabled"`, `"metadata"`, `"contents"`.

### **`allow_parameter_qos_overrides`**

Type: `bool` Default: `true`

:   Whether to allow QoS profiles to be overwritten with parameterized values.

### **`footprint_padding`**

Type: `double` Default: `0.01`

:   Amount to pad footprint (m).

### **`footprint`**

Type: `vector<double>` Default: `[]`

:   Ordered set of footprint points passed in as a string, must be closed set. For example, the following defines a square base with side lengths of 0.2 meters `footprint: "[ [0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1] ]"`. Note that this can also be adjusted over time using the costmap's `~/footprint` topic, which will update the polygon over time as needed due to changes in the robot's state, such as movement of an attached manipulator, picking up a pallet, or other actions that adjust a robot's shape. If this parameter is set, `isPathValid` will do full collision checking.

### **`global_frame`**

Type: `string` Default: `"map"`

:   Reference frame.

### **`height`**

Type: `int` Default: `5`

:   Height of costmap (m).

### **`width`**

Type: `int` Default: `5`

:   Width of costmap (m).

### **`lethal_cost_threshold`**

Type: `int` Default: `100`

:   Minimum cost of an occupancy grid map to be considered a lethal obstacle.

### **`map_vis_z`**

Type: `double` Default: `0.0`

:   The height of the map used for visualization, helping to avoid RViz flickering issues (e.g., at -0.008).

### **`origin_x`**

Type: `double` Default: `0.0`

:   X origin of the costmap relative to width (m).

### **`origin_y`**

Type: `double` Default: `0.0`

:   Y origin of the costmap relative to height (m).

### **`publish_frequency`**

Type: `double` Default: `1.0`

:   Frequency (Hz) at which the costmap is published to a topic.
    Higher values provide more frequent updates for visualization and debugging but increase bandwidth usage.

### **`resolution`**

Type: `double` Default: `0.1`

:   Resolution of each cell (pixel) in the costmap, in meters.
    Smaller values increase map accuracy and obstacle detail but require more computation.
    Larger values reduce computational load but may miss fine obstacles.

### **`robot_base_frame`**

Type: `string` Default: `"base_link"`

:   Robot base frame.

### **`robot_radius`**

Type: `double` Default: `0.1`

:   Robot radius to use, if footprint coordinates not provided. If this parameter is set, `isPathValid` will do circular collision checking.

### **`subscribe_to_stamped_footprint`**

Type: `bool` Default: `false`

:   If true, the costmap will subscribe to PolygonStamped footprint messages instead of Polygon messages. This allows the footprint to include timestamp and frame information, which can be useful for applications that need temporally-aware footprint data.

### **`rolling_window`**

Type: `bool` Default: `false`

:   If true, the costmap moves with the robot, maintaining a local view centered around it.
    This is typically used for local costmaps.
    If false, the costmap remains fixed in the global frame.

### **`track_unknown_space`**

Type: `bool` Default: `false`

:   If false, treats unknown space as free space, else as unknown space.

### **`transform_tolerance`**

Type: `double` Default: `0.3`

:   TF transform tolerance.

### **`initial_transform_timeout`**

Type: `double` Default: `60.0`

:   Time to wait for the transform from robot base frame to global frame to become available. If exceeded, the  configuration stage is aborted.

### **`trinary_costmap`**

Type: `bool` Default: `true`

:   If occupancy grid map should be interpreted as only 3 values (free, occupied, unknown) or with its stored values.

### **`unknown_cost_value`**

Type: `int` Default: `255`

:   Cost of unknown space if tracking it.

### **`inscribed_obstacle_cost_value`**

Type: `int` Default: `99`

:   The OccupancyGrid values that represents `INSCRIBED_INFLATED_OBSTACLE` during costmap conversion operations.

### **`update_frequency`**

Type: `double` Default: `5.0`

:   Costmap update frequency.

### **`use_maximum`**

Type: `bool` Default: `false`

:   Whether to use the maximum cost when combining multiple costmap layers.
    If true, the highest cost is preserved, ensuring obstacles are not overwritten.
    If false, newer layers may override previous cost values.

### **`plugins`**

Type: `vector<string>` Default: `["static_layer", "obstacle_layer", "inflation_layer"]`

:   List of mapped plugin names for parameter namespaces and names.

    Note
    :   Each plugin namespace defined in this list needs to have a `plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:
        ```yaml
        local_costmap:
          ros__parameters:
            plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
            obstacle_layer:
              plugin: "nav2_costmap_2d::ObstacleLayer"
            voxel_layer:
              plugin: "nav2_costmap_2d::VoxelLayer"
            inflation_layer:
              plugin: "nav2_costmap_2d::InflationLayer"
        ```

### **`filters`**

Type: `vector<string>` Default: `[""]`

:   List of mapped costmap filter names for parameter namespaces and names.

    Note
    :   Costmap filters are also loadable plugins just as ordinary costmap layers. This separation is made to avoid plugin and filter interference and places these filters on top of the combined layered costmap. As  with plugins, each costmap filter namespace defined in this list needs to have a `plugin` parameter defining the type of filter plugin to be loaded in the namespace.

        Example:
        ```yaml
        local_costmap:
          ros__parameters:
            filters: ["keepout_filter", "speed_filter"]
            keepout_filter:
              plugin: "nav2_costmap_2d::KeepoutFilter"
            speed_filter:
              plugin: "nav2_costmap_2d::SpeedFilter"
        ```

## Default Plugins

When the `plugins` parameter is not overridden, the following default plugins are loaded:

| Namespace         | Plugin                            |
|-------------------|-----------------------------------|
| "static_layer"    | "nav2_costmap_2d::StaticLayer"    |
| "obstacle_layer"  | "nav2_costmap_2d::ObstacleLayer"  |
| "inflation_layer" | "nav2_costmap_2d::InflationLayer" |

## Plugin Parameters

<div class="grid" markdown>

[Static Layer Parameters][static-layer-parameters]{ .md-button .md-button--primary }
[Inflation Layer Parameters][inflation-layer-parameters]{ .md-button .md-button--primary }
[Legacy Inflation Layer Parameters][legacy-inflation-layer-parameters]{ .md-button .md-button--primary }
[Obstacle Layer Parameters][obstacle-layer-parameters]{ .md-button .md-button--primary }
[Voxel Layer Parameters][voxel-layer-parameters]{ .md-button .md-button--primary }
[Range Sensor Parameters][range-sensor-parameters]{ .md-button .md-button--primary }
[Denoise Layer Parameters][denoise-layer-parameters]{ .md-button .md-button--primary }
[Plugin Container Layer Parameters][plugin-container-layer-parameters]{ .md-button .md-button--primary }

</div>

## Costmap Filters Parameters

<div class="grid" markdown>

[Keepout Filter Parameters][keepout-filter-parameters]{ .md-button .md-button--primary }
[Speed Filter Parameters][speed-filter-parameters]{ .md-button .md-button--primary }
[Binary Filter Parameters][binary-filter-parameters]{ .md-button .md-button--primary }

</div>

## Example

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      footprint_padding: 0.03
      update_frequency: 1.0
      publish_frequency: 1.0
      transform_tolerance: 0.1
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22 # radius set and used, so no footprint points
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        footprint_clearing_enabled: true
        max_obstacle_height: 2.0
        combination_method: 1
        scan:
          topic: /scan
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          inf_is_valid: false
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        footprint_clearing_enabled: true
        max_obstacle_height: 2.0
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        unknown_threshold: 15
        mark_threshold: 0
        observation_sources: pointcloud
        combination_method: 1
        pointcloud:  # no frame set, uses frame from message
          topic: /intel_realsense_r200_depth/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
          transport_type: "raw"  # raw or/ with compression (zlib, draco, zstd)
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        enabled: true
        subscribe_to_updates: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.55
        cost_scaling_factor: 1.0
        inflate_unknown: false
        inflate_around_unknown: true
      always_send_full_costmap: True
      introspection_mode: "disabled"


local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      introspection_mode: "disabled"
```
