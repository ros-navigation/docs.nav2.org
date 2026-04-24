# ToggleCollisionMonitor { #toggle-collision-monitor }

Calls the ToggleCollisionMonitor service. Used to toggle the collision monitor on (enabled) and off (disabled).

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/toggle_collision_monitor_service.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
