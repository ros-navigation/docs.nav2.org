# GetPoseFromPath { #get-pose-from-path }

Gets a pose from a particular index on the path. Use `-1` to get the last pose, `-2` for second to last, and so on.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/get_pose_from_path_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
