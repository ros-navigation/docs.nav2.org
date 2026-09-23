# Keepout Filter Parameters { #keepout-filter-parameters }

Keepout Filter - is a Costmap Filter that enforces robot to avoid keepout areas or stay on preferred lanes, by updating corresponding costmap layer using filter mask information.

You may use this filter with an inflation layer to inflate the keepout costs around the keepout zone. This helps planners and controllers account for contact with any part of the robot footprint, rather than only its center. If `override_lethal_cost` is enabled, it allows the robot to navigate out after it enters a keepout zone. If you plan on using the keepout filter with a planner or controller that has SE2 check enabled, the keepout filter should have inflation, otherwise the robot can get stuck on the edge of the keepout.

`<filter name>`: is the corresponding plugin name selected for this type.

### **`<filter name>.enabled`**

Type: `bool` Default: `true`

:   Whether it is enabled.

### **`<filter name>.filter_info_topic`**

Type: `string` Default: `N/A`

:   Name of the incoming [CostmapFilterInfo](https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/msg/CostmapFilterInfo.msg) topic having filter-related information. Published by Costmap Filter Info Server along with filter mask topic. For more details about Map and Costmap Filter Info servers configuration please refer to the [Map Server][map-server-index] configuration page.

### **`<filter name>.override_lethal_cost`**

Type: `bool` Default: `false`

:   When `true`, check if the robot is in a lethal keepout zone, if so, replaces those lethal costs with `lethal_override_cost`.

### **`<filter name>.lethal_override_cost`**

Type: `double` Default: `252.0`

:   The cost value written into those cells instead of lethal cost when override is active. Default sets cost very high to incentivize leaving the area as soon as possible.

### **`<filter name>.transform_tolerance`**

Type: `double` Default: `0.1`

:   Time with which to post-date the transform that is published, to indicate that this transform is valid into the future. Used when filter mask and current costmap layer are in different frames.

## Example

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      ...
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      filters: ["keepout_filter"]
      ...
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: True
        filter_info_topic: "/costmap_filter_info"
        transform_tolerance: 0.1
        override_lethal_cost: True
        lethal_override_cost: 200
...
local_costmap:
  local_costmap:
    ros__parameters:
      ...
      plugins: ["voxel_layer", "inflation_layer"]
      filters: ["keepout_filter"]
      ...
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: True
        filter_info_topic: "/costmap_filter_info"
        transform_tolerance: 0.1
        override_lethal_cost: True
        lethal_override_cost: 200
```
