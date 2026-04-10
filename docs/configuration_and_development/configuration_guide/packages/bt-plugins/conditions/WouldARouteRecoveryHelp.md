# WouldARouteRecoveryHelp { #would-a-route-recovery-help }

Checks if the active route server error code is UNKNOWN, NO_VALID_ROUTE, or TIMEOUT.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<WouldARouteRecoveryHelp error_code="{compute_route_to_pose_error_code}"/>
```
