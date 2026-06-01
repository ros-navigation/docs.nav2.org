# ClearCostmapExceptRegion { #clear-costmap-except-region }

Action to call a costmap clearing except region server.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/clear_costmap_service.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path, "ClearCostmapExceptRegionService") }}
