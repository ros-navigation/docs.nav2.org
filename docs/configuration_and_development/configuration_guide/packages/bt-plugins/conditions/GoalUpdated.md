# GoalUpdated { #goal-updated }

Checks if the global navigation goal, or a vector of goals, has changed in the blackboard.
Returns failure if the goal is the same, if it changes, it returns success.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/condition/goal_updated_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
