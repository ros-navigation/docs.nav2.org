# IsBatteryLow { #is-battery-low }

Checks if battery is low by subscribing to a `sensor_msgs/BatteryState` topic and checking if battery percentage/voltage is below a specified minimum value.
By default percentage (in range 0 to 1) is used to check for low battery. Set the `is_voltage` parameter to true to use voltage.
Returns SUCCESS when battery percentage/voltage is lower than the specified value, FAILURE otherwise.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<IsBatteryLow min_battery="0.5" battery_topic="/battery_status" is_voltage="false"/>
```
