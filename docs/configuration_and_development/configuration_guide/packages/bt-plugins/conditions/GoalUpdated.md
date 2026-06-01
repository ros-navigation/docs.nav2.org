# GoalUpdated { #goal-updated }

Checks if the global navigation goal, or a vector of goals, has changed in the blackboard.
Returns failure if the goal is the same, if it changes, it returns success.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/condition/goal_updated_condition.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
