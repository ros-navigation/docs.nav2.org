# NavigateThroughPoses { #navigate-through-poses }

Invokes the NavigateThroughPoses ROS 2 action server, which is implemented by the [bt_navigator](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bt_navigator) module.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/navigate_through_poses_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
