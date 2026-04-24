# ClearEntireCostmap { #clear-entire-costmap }

Action to call a costmap clearing server.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/clear_costmap_service.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path, "ClearEntireCostmapService") }}
