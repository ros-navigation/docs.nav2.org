# CheckStopStatus { #check-stop-status }

BT node that tracks robot odometry and returns SUCCESS if robot is considered stopped for long enough,
RUNNING if stopped but not for long enough and FAILURE otherwise

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/check_stop_status_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
