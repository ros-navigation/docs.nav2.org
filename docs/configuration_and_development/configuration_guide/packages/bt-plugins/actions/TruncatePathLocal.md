# TruncatePathLocal { #truncate-path-local }

A custom control node, which modifies a path making it shorter. It removes parts of the path which are more distant than specified forward/backward distance around robot

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/truncate_path_local_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
