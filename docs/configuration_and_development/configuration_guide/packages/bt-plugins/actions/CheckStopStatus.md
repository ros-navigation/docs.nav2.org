# CheckStopStatus { #check-stop-status }

BT node that tracks robot odometry and returns SUCCESS if robot is considered stopped for long enough,
RUNNING if stopped but not for long enough and FAILURE otherwise

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<CheckStopStatus velocity_threshold="0.01" duration_stopped="1000"/>
```
