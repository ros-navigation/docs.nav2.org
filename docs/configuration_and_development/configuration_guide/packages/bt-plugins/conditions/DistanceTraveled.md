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

```xml
<DistanceTraveled distance="0.8" global_frame="map" robot_base_frame="base_link"/>
```
