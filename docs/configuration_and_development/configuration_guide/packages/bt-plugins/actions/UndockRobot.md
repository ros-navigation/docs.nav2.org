# UndockRobot { #undock-robot }

Invokes the UndockRobot ROS 2 action server, which is implemented by the docking server.

It is used to undock the robot from a docking station.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_docking_hpp_path + "/undock_robot.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
