# GetNextFewGoals { #get-next-few-goals }

Extracts only the next `N` goals from a list of goals to send to a later task that only needs localized future knowledge.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<GetNextFewGoals num_goals="3" input_goals="{goal_poses}" output_goals="{planning_goals}"/>
```
