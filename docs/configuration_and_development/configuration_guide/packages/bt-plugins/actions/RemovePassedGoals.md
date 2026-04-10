# RemovePassedGoals { #remove-passed-goals }

Looks over the input port `goals` and removes any point that the robot is in close proximity to or has recently passed.
This is used to cull goal points that have been passed from `ComputePathThroughPoses` to enable replanning to only the current task goals.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<RemovePassedGoals radius="0.6" input_goals="{goals}" output_goals="{goals}" input_waypoint_statuses="{waypoint_statuses}" output_waypoint_statuses="{waypoint_statuses}"/>
```
