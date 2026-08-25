# IsWithinPathTrackingBounds { #is-within-path-tracking-bounds }

Checks if the robot is within determined tracking error bounds during path following.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/condition/is_within_path_tracking_bounds_condition.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
