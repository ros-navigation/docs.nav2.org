# ClearEntireCostmap { #clear-entire-costmap }

Action to call a costmap clearing server.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/clear_costmap_service.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path, "ClearEntireCostmapService") }}
