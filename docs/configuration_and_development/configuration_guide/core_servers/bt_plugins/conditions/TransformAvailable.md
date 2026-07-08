# TransformAvailable { #transform-available }

Checks if a TF transform is available. Returns failure if it cannot be found. Once found, it will always return success. Useful for initial condition checks.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/condition/transform_available_condition.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
