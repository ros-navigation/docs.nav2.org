# GetNextFewGoals { #get-next-few-goals }

Extracts only the next `N` goals from a list of goals to send to a later task that only needs localized future knowledge.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/get_next_few_goals_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
