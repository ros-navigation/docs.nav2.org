# Static Layer Parameters { #static-layer-parameters }

This implements a costmap layer taking in a map from either SLAM or `map_server` (or other) to place into the costmap. It resizes the costmap to its size and places the static obstacles on the planning space.

`<static layer>` is the corresponding plugin name selected for this type.

### **`<static layer>.enabled`**

Type: `bool` Default: `True`

:   Whether it is enabled.

### **`<static layer>.footprint_clearing_enabled`**

Type: `bool` Default: `False`

:   Clear any occupied cells under robot footprint.

### **`<static layer>.subscribe_to_updates`**

Type: `bool` Default: `False`

:   Subscribe to static map updates after receiving first.

### **`<static layer>.map_subscribe_transient_local`**

Type: `bool` Default: `True`

:   QoS settings for map topic.

### **`<static layer>.map_topic`**

Type: `string` Default: `""`

:   Map topic to subscribe to. If left empty the map topic will default to the global `map_topic` parameter in `costmap_2d_ros`.
