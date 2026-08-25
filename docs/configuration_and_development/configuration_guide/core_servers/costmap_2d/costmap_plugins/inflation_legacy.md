# Legacy Inflation Layer Parameters { #legacy-inflation-layer-parameters }

!!! warning

    This layer has been deprecated in favor of the refactored [Inflation Layer][inflation-layer-parameters], which offers significantly improved performance (up to 11.5× faster with OpenMP enabled). Consider migrating to the new implementation.

This layer places an exponential decay functions around obstacles to increase cost to traverse near collision. It also places a lethal cost around obstacles within the robot's fully inscribed radius - even if a robot is non-circular for optimized first-order collision checking.

`<inflation layer>` is the corresponding plugin name selected for this type.

### **`<inflation layer>.enabled`**

Type: `bool` Default: `true`

:   Whether it is enabled.

### **`<inflation layer>.inflation_radius`**

Type: `double` Default: `0.55`

:   Radius to inflate costmap around lethal obstacles.

### **`<inflation layer>.cost_scaling_factor`**

Type: `double` Default: `10.0`

:   Exponential decay factor across inflation radius.

### **`<inflation layer>.inflate_unknown`**

Type: `bool` Default: `false`

:   Whether to inflate unknown cells as if lethal.

### **`<inflation layer>.inflate_around_unknown`**

Type: `bool` Default: `false`

:   Whether to inflate unknown cells.
