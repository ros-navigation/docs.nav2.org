# RemovePassedGoals { #remove-passed-goals }

Looks over the input port `goals` and removes any point that the robot is in close proximity to or has recently passed.
This is used to cull goal points that have been passed from `ComputePathThroughPoses` to enable replanning to only the current task goals.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/remove_passed_goals_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
