# PathExpiringTimer { #path-expiring-timer }

Checks if the timer has expired. Returns success if the timer has expired, otherwise it returns failure.
The timer will reset if the path gets updated.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/condition/path_expiring_timer_condition.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
