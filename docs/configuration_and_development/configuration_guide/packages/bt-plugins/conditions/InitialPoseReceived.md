# InitialPoseReceived { #initial-pose-received }

Node that returns success when the initial pose is sent to AMCL via `/initial_pose`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/condition/initial_pose_received_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
