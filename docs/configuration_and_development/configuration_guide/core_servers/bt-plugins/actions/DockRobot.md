# DockRobot { #dock-robot }

Invokes the DockRobot ROS 2 action server, which is implemented by the docking server.

It is used to dock the robot to a docking station.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_docking_hpp_dir_path + "/dock_robot.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
