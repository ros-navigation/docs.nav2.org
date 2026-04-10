# GoalReached { #goal-reached }

Checks the distance to the goal, if the distance to goal is less than the pre-defined threshold, the tree returns SUCCESS, otherwise it returns FAILURE.

## Parameter

### **`transform_tolerance`**

  Defined and declared in [Behavior-Tree Navigator][behavior-tree-navigator].

### **`goal_reached_tol`**

| Type     | Default |
|----------|---------|
| `double` | 0.25    |

Description
:   Tolerance of accepting pose as the goal (m).

## Example

```yaml
bt_navigator:
  ros__parameters:
    # other bt_navigator parameters
    transform_tolerance: 0.1
    goal_reached_tol: 0.25
```

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<GoalReached goal="{goal}" robot_base_frame="base_link"/>
```
