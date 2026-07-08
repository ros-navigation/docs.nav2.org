# TruncatePath { #truncate-path }

A custom control node, which modifies a path making it shorter. It removes parts of the path closer than a distance to the goal pose. The resulting last pose of the path orientates the robot to the original goal pose.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/truncate_path_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
