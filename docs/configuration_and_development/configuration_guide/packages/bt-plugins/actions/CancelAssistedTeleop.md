# CancelAssistedTeleop { #cancel-assisted-teleop }

Used to cancel the AssistedTeleop action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/assisted_teleop_cancel_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
