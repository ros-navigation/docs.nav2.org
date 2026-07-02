# ArePosesNear { #are-poses-near }

Checks if two poses are nearby. If the input poses are in different frames, it will automatically transform both to the global frame.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/condition/are_poses_near_condition.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
