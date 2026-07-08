# CheckStopStatus { #check-stop-status }

BT node that tracks robot odometry and returns SUCCESS if robot is considered stopped for long enough,
RUNNING if stopped but not for long enough and FAILURE otherwise

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/check_stop_status_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
