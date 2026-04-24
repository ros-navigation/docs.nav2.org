# ArePosesNear { #are-poses-near }

Checks if two poses are nearby. If the input poses are in different frames, it will automatically transform both to the global frame.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/condition/are_poses_near_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
