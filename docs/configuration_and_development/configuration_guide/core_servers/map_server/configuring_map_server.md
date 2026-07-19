# Map Server { #map-server }

The Map Server implements the server for handling the map load requests for the stack and hosts a map topic.

## Map Server Parameters

### **`yaml_filename`**

Type: `string` Default: `N/A`

:   Path to map yaml file. This parameter can set either from the yaml file or using the launch configuration parameter `map`. If we set it on launch commandline / launch configuration default, we override the yaml default. If you would like the specify your map file in yaml, remove the launch default so it is not overridden in Nav2's default launch files.

### **`topic_name`**

Type: `string` Default: `"map"`

:   Topic to publish loaded map to.

### **`frame_id`**

Type: `string` Default: `"map"`

:   Frame to publish loaded map in.

### **`introspection_mode`**

Type: `string` Default: `"disabled"`

:   The introspection mode for services and actions. Options are `"disabled"`, `"metadata"`, `"contents"`.
