# AppendGoalPoseToGoals { #append-goal-pose-to-goals }

Appends a goal `PoseStamped` to the end of a `goals` vector.
May be useful to add in the final task goal pose to a list of goals extracted from Route nodes (or other sources of future goals).

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/append_goal_pose_to_goals_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
