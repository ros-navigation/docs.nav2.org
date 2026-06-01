# GetCurrentPose { #get-current-pose }

Obtains the current pose from TF and places it on the blackboard for other nodes to use.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/get_current_pose_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
