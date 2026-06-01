# CancelControl { #cancel-control }

Used to cancel the goals given to the controllers' action server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/controller_cancel_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
