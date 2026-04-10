# PathExpiringTimer { #path-expiring-timer }

Checks if the timer has expired. Returns success if the timer has expired, otherwise it returns failure.
The timer will reset if the path gets updated.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<PathExpiringTimer seconds="15" path="{path}"/>
```
