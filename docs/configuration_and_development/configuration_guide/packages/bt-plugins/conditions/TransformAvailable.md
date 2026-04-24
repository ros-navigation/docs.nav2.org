# TransformAvailable { #transform-available }

Checks if a TF transform is available. Returns failure if it cannot be found. Once found, it will always return success. Useful for initial condition checks.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/condition/transform_available_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
