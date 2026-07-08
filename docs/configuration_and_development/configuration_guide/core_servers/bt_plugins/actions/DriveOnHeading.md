# DriveOnHeading { #drive-on-heading }

Invokes the DriveOnHeading ROS 2 action server, which causes the robot to drive on the current heading by a specific displacement.
It performs a linear translation by a given distance. The nav2_behaviors module implements the DriveOnHeading action server.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/drive_on_heading_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
