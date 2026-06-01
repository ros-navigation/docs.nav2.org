# SingleTrigger { #single-trigger }

This node triggers its child only once and returns FAILURE for every succeeding tick.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/decorator/single_trigger_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
