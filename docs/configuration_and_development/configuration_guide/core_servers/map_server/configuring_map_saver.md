# Map Saver { #map-saver }

The map saver server runs in the background and saves maps based on service requests. There exists a map saver CLI similar to ROS 1 as well for a single map save.

## Map Saver Parameters

### **`save_map_timeout`**

Type: `int` Default: `2`

:   Timeout to attempt saving the map (seconds).

### **`free_thresh_default`**

Type: `double` Default: `0.25`

:   Free space maximum probability threshold value for occupancy grid.

### **`occupied_thresh_default`**

Type: `double` Default: `0.65`

:   Occupied space minimum probability threshold value for occupancy grid.

### **`introspection_mode`**

Type: `string` Default: `"disabled"`

:   The introspection mode for services and actions. Options are "disabled", "metadata", "contents".
