# GoalUpdatedController { #goal-updated-controller }

Checks if the global navigation goal, or a vector of goals, has changed in the blackboard. The node ticks its child if the goal was updated.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/decorator/goal_updated_controller.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
