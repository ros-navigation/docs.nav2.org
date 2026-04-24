# RoundRobin { #round-robin }

Custom control flow node used to create a round-robin behavior for children BT nodes.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/control/round_robin_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
