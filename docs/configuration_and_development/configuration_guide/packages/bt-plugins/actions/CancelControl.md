# CancelControl { #cancel-control }

Used to cancel the goals given to the controllers’ action server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<CancelControl server_name="FollowPath" server_timeout="10"/>
```
