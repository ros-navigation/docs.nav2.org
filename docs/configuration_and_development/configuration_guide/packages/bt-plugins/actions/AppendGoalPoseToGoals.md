# AppendGoalPoseToGoals { #append-goal-pose-to-goals }

Appends a goal `PoseStamped` to the end of a `goals` vector.
May be useful to add in the final task goal pose to a list of goals extracted from Route nodes (or other sources of future goals).

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<AppendGoalPoseToGoals goal_pose="{goal}" input_goals="{goal_poses}" output_goals="{goal_poses}"/>
```
