# RemoveInCollisionGoals { #remove-in-collision-goals }

Looks over the input port `goals` and removes any waypoint that has a point or footprint cost above a certain threshold.
This may be used to cull goal points passed from `ComputePathThroughPoses` to avoid waiting indefinitely on occupied waypoints.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<RemoveInCollisionGoals input_goals="{goals}" output_goals="{goals}" cost_threshold="254.0" use_footprint="true" service_name="/global_costmap/get_cost_global_costmap" input_waypoint_statuses="{waypoint_statuses}" output_waypoint_statuses="{waypoint_statuses}" />
```
