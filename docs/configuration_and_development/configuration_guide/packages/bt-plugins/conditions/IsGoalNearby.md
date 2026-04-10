# IsGoalNearby { #is-goal-nearby }

Checks if the robot is near the goal by computing the remaining path length from the robot’s current position to the goal. Returns SUCCESS when the remaining path length is less than the proximity threshold, otherwise returns FAILURE.

## Parameter

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
<IsGoalNearby path="{path}" proximity_threshold="1.0" />
```
