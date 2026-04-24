# ClearCostmapAroundRobot { #clear-costmap-around-robot }

Action to call a costmap clearing around robot server.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/clear_costmap_service.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path, "ClearCostmapAroundRobotService") }}
