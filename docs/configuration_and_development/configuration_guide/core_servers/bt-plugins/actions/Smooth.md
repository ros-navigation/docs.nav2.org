# SmoothPath { #smooth-path }

Invokes the SmoothPath action API in the smoother server to smooth a given path plan.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/smooth_path_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
