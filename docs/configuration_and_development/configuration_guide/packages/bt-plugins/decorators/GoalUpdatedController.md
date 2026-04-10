# GoalUpdatedController { #goal-updated-controller }

Checks if the global navigation goal, or a vector of goals, has changed in the blackboard. The node ticks its child if the goal was updated.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<GoalUpdatedController goal="{goal}" goals="{goals}">
  <!--Add tree components here-->
</GoalUpdatedController>
```
