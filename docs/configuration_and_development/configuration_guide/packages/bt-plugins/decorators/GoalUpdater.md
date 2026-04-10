# GoalUpdater { #goal-updater }

A custom control node, which updates the goal(s) pose(s). It subscribes to a topic in which it can receive (an) updated goal(s) pose(s) to use instead of the one(s) commanded in action. It is useful for dynamic object following tasks.

## Parameters

### **`goal_updater_topic`**

| Type     | Default       |
|----------|---------------|
| `string` | “goal_update” |

Description
:   The topic to receive the updated goal pose

### **`goals_updater_topic`**

| Type     | Default        |
|----------|----------------|
| `string` | “goals_update” |

Description
:   The topic to receive the updated goals poses

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<GoalUpdater input_goal="{goal}" input_goals="{goals}" output_goal="{goal}" output_goals="{goals}">
  <!--Add tree components here-->
</GoalUpdater>
```
