# IsWithinPathTrackingBounds { #is-within-path-tracking-bounds }

Checks if the robot is within determined tracking error bounds during path following.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/condition/is_within_path_tracking_bounds_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
