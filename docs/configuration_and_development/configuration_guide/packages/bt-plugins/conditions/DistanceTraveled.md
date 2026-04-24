# DistanceTraveled { #distance-traveled }

Node that returns success when a configurable distance has been traveled.

## Parameters

### **`transform_tolerance`**

  Defined and declared in [Behavior-Tree Navigator][behavior-tree-navigator].

## Example

```yaml
bt_navigator:
  ros__parameters:
    # other bt_navigator parameters
    transform_tolerance: 0.1
```

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/condition/distance_traveled_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
