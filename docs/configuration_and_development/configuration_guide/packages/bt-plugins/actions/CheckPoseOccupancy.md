# CheckPoseOccupancy { #check-pose-occupancy }

Checks to see if the pose is occupied. If it is occupied, it returns SUCCESS, otherwise
it returns FAILURE.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/check_pose_occupancy_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
