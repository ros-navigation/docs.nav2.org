# Range Sensor Parameters { #range-sensor-parameters }

This costmap layer implements a plugin that processes sonar, IR, or other 1-D sensors for collision avoidance.

`<range layer>` is the corresponding plugin name selected for this type.

### **`<range layer>.enabled`**

Type: `bool` Default: `true`

:   Whether it is enabled.

### **`<range layer>.topics`**

Type: `vector<string>` Default: `[""]`

:   Range topics to subscribe to.

    Relative topics will be relative to the node's parent namespace.
    For example, if you specify `topics: [range1, /range2]` in the `range_layer` of a `local_costmap` and you launch your bringup with a `tb4` namespace:

      - User chosen namespace is `tb4`.
      - User chosen topics are `[range1, /range2]`.
      - Topic will be remapped to `/tb4/range1`, without `local_costmap`, and `/range2`.
      - Use global topics such as `/range2` if you do not wish the node namespace to apply.

### **`<range layer>.phi`**

Type: `double` Default: `1.2`

:   Phi value.

### **`<range layer>.inflate_cone`**

Type: `double` Default: `1.0`

:   Inflate the triangular area covered by the sensor (percentage).

### **`<range layer>.no_readings_timeout`**

Type: `double` Default: `0.0`

:   If zero, this parameter has no effect. Otherwise if the layer does
    not receive sensor data for this amount of time,
    the layer will warn the user and the layer will be marked as not current.

### **`<range layer>.clear_threshold`**

Type: `double` Default: `0.2`

:   Probability below which cells are marked as free.

### **`<range layer>.mark_threshold`**

Type: `double` Default: `0.8`

:   Probability above which cells are marked as occupied.

### **`<range layer>.clear_on_max_reading`**

Type: `bool` Default: `false`

:   Whether to clear the sensor readings on max range.

### **`<range layer>.input_sensor_type`**

Type: `string` Default: `"ALL"`

:   Input sensor type is either `"ALL"` (automatic selection), `"VARIABLE"` (min range != max range), or `"FIXED"` (min range == max range).
