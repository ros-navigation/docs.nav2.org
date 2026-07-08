# IsBatteryCharging { #is-battery-charging }

Checks if the battery is charging by subscribing to a `sensor_msgs/BatteryState` topic and checking if the power_supply_status is `POWER_SUPPLY_STATUS_CHARGING`.
Returns SUCCESS in that case, FAILURE otherwise.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/condition/is_battery_charging_condition.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
