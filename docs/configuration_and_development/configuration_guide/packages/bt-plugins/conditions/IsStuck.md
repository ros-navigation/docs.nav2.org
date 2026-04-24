# IsStuck { #is-stuck }

Determines if the robot is not progressing towards the goal.
If the robot is stuck and not progressing, the condition returns
SUCCESS, otherwise it returns FAILURE.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/condition/is_stuck_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
