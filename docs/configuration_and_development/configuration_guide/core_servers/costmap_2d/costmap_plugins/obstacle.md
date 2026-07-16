# Obstacle Layer Parameters { #obstacle-layer-parameters }

This costmap layer implements a plugin that uses 2D raycasting for 2D lidars, depth, or other sensors. It contains a 2D costmap model within it that manages the planning space by the parameters specified below.

`<obstacle layer>` is the corresponding plugin name selected for this type.

`<data source>` is the corresponding observation source name for that sources parameters.

### **`<obstacle layer>.enabled`**

Type: `bool` Default: `true`

:   Whether it is enabled.

### **`<obstacle layer>.footprint_clearing_enabled`**

Type: `bool` Default: `true`

:   Clear any occupied cells under robot footprint.

### **`<obstacle layer>.min_obstacle_height`**

Type: `double` Default: `0.0`

:   Minimum height to add return to occupancy grid.

### **`<obstacle layer>.max_obstacle_height`**

Type: `double` Default: `2.0`

:   Maximum height to add return to occupancy grid.

### **`<obstacle layer>.combination_method`**

Type: `int` Default: `1`

:   Enum for method to add data to master costmap. Must be 0, 1 or 2, default to 1 (see below).

    0 - Overwrite: Overwrite master costmap with every valid observation.

    1 - Max: Sets the new value to the maximum of the master_grid's value and this layer's value. This is the default.

    2 - MaxWithoutUnknownOverwrite: Sets the new value to the maximum of the master_grid's value and this layer's value. If the master value is NO_INFORMATION, it is NOT overwritten.
    It can be used to make sure that the static map is the dominant source of information, and prevent the robot to go through places that are not present in the static map.

### **`<obstacle layer>.tf_filter_tolerance`**

Type: `double` Default: `0.05`

:   Tolerance for the `tf2_ros::MessageFilter`.

### **`<obstacle layer>.observation_sources`**

Type: `vector<string>` Default: `{""}`

:   namespace of sources of data.

### **`<obstacle layer>.<data source>.topic`**

Type: `string` Default: `""`

:   Topic of data.

    Relative topics will be relative to the node's parent namespace.
    For example, if you specify *topic: scan* in the *obstacle_layer* of a *local_costmap* and you launch your bringup with a *tb4* namespace:

      - User chosen namespace is *tb4*.
      - User chosen topic is *scan*.
      - Topic will be remapped to */tb4/scan* without *local_costmap*.
      - Use global topic */scan* if you do not wish the node namespace to apply.

### **`<obstacle layer>.<data source>.sensor_frame`**

Type: `string` Default: `""`

:   Frame of sensor, to use if not provided by message. If empty, uses message frame_id.

### **`<obstacle layer>.<data source>.observation_persistence`**

Type: `double` Default: `0.0`

:   How long to store messages in a buffer to add to costmap before removing them (s).

### **`<obstacle layer>.<data source>.expected_update_rate`**

Type: `double` Default: `0.0`

:   Expected rate to get new data from sensor.

### **`<obstacle layer>.<data source>.data_type`**

Type: `string` Default: `"LaserScan"`

:   Data type of input, LaserScan or PointCloud2.

### **`<obstacle layer>.<data source>.transport_type`**

Type: `string` Default: `"raw"`

:   For `PointCloud2` data, specify the transport plugin to use:

    - raw: No compression. Default; highest bandwidth usage.
    - draco: Lossy compression via Google.
    - zlib: Lossless compression via Zlib compression.
    - zstd: Lossless compression via Zstd compression.

  See the [known transports](https://github.com/ros-perception/point_cloud_transport_plugins) for more details.

### **`<obstacle layer>.<data source>.min_obstacle_height`**

Type: `double` Default: `0.0`

:   Minimum height to add return to occupancy grid.

### **`<obstacle layer>.<data source>.max_obstacle_height`**

Type: `double` Default: `0.0`

:   Maximum height to add return to occupancy grid.

### **`<obstacle layer>.<data source>.inf_is_valid`**

Type: `bool` Default: `false`

:   Are infinite returns from laser scanners valid measurements to raycast.

### **`<obstacle layer>.<data source>.marking`**

Type: `bool` Default: `true`

:   Whether source should mark in costmap.

### **`<obstacle layer>.<data source>.clearing`**

Type: `bool` Default: `false`

:   Whether source should raytrace clear in costmap.

### **`<obstacle layer>.<data source>.obstacle_max_range`**

Type: `double` Default: `2.5`

:   Maximum range to mark obstacles in costmap.

### **`<obstacle layer>.<data source>.obstacle_min_range`**

Type: `double` Default: `0.0`

:   Minimum range to mark obstacles in costmap.

### **`<obstacle layer>.<data source>.raytrace_max_range`**

Type: `double` Default: `3.0`

:   Maximum range to raytrace clear obstacles from costmap.

### **`<obstacle layer>.<data source>.raytrace_min_range`**

Type: `double` Default: `0.0`

:   Minimum range to raytrace clear obstacles from costmap.
