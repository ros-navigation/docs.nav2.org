# CancelSpin { #cancel-spin }

Used to cancel the spin action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/spin_cancel_node.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
