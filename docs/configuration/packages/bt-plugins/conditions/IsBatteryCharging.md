<a id="bt-is-battery-charging-condition"></a>

# IsBatteryCharging

Checks if the battery is charging by subscribing to a `sensor_msgs/BatteryState` topic and checking if the power_supply_status is `POWER_SUPPLY_STATUS_CHARGING`.
Returns SUCCESS in that case, FAILURE otherwise.

## Input Ports

* **battery_topic:**
  | Type   | Default           |
  |--------|-------------------|
  | string | “/battery_status” |

  Description
  : Topic for battery info.

## Example

```xml
<IsBatteryCharging battery_topic="/battery_status"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
