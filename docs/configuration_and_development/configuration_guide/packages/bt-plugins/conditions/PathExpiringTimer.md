# PathExpiringTimer { #path-expiring-timer }

Checks if the timer has expired. Returns success if the timer has expired, otherwise it returns failure.
The timer will reset if the path gets updated.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/condition/path_expiring_timer_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
