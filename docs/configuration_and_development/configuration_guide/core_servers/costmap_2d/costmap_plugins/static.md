# Static Layer Parameters { #static-layer-parameters }

This implements a costmap layer taking in a map from either SLAM or `map_server` (or other) to place into the costmap. By default, it resizes a non-rolling costmap to match the incoming map and places the static obstacles on the planning space. Set `resize_master` to `false` to add a map as an overlay without resizing the master costmap.

`<static layer>` is the corresponding plugin name selected for this type.

### **`<static layer>.transform_staleness_threshold`**

Type: `double` Default: `0.0`

:   Maximum age (seconds) of the latest transform used to place the static map in the costmap. Values greater than `0.0` enable the age check; non-positive values disable it.

### **`<static layer>.enabled`**

Type: `bool` Default: `true`

:   Whether it is enabled.

### **`<static layer>.resize_master`**

Type: `bool` Default: `true`

:   Resize a non-rolling master costmap to match the incoming map. When `false`, overlay the map without changing the master's geometry.

### **`<static layer>.footprint_clearing_enabled`**

Type: `bool` Default: `false`

:   Clear any occupied cells under robot footprint.

### **`<static layer>.restore_cleared_footprint`**

Type: `bool` Default: `true`

:   Restore map after clearing the area the footprint occupied.
    If `footprint_clearing_enabled` is `false`, this parameter is ignored.

### **`<static layer>.subscribe_to_updates`**

Type: `bool` Default: `false`

:   Subscribe to static map updates after receiving first.

### **`<static layer>.map_subscribe_transient_local`**

Type: `bool` Default: `true`

:   QoS settings for map topic.

### **`<static layer>.map_topic`**

Type: `string` Default: `"map"`

:   Map topic to subscribe to.

    Relative topics will be relative to the node's parent namespace.
    For example, if you specify `map_topic: map` in the `static_layer` of a `global_costmap` and you launch your bringup with a `tb4` namespace:

      - User chosen namespace is `tb4`.
      - User chosen topic is `map`.
      - Topic will be remapped to `/tb4/map` without `global_costmap`.
      - Use global topic `/map` if you do not wish the node namespace to apply.
