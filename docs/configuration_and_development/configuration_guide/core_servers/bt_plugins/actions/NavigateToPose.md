# NavigateToPose { #navigate-to-pose }

Invokes the NavigateToPose ROS 2 action server, which is implemented by the [bt_navigator](https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator) module.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/navigate_to_pose_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
