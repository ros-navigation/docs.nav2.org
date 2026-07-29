# ComputeAndTrackRoute { #compute-and-track-route }

Invokes the ComputeAndTrackRoute ROS 2 action server, which is implemented by the [nav2_route](https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_route) module.
The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/compute_and_track_route_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
