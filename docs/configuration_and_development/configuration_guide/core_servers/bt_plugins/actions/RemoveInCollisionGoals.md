# RemoveInCollisionGoals { #remove-in-collision-goals }

Looks over the input port `goals` and removes any waypoint that has a point or footprint cost above a certain threshold.
This may be used to cull goal points passed from `ComputePathThroughPoses` to avoid waiting indefinitely on occupied waypoints.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/remove_in_collision_goals_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
