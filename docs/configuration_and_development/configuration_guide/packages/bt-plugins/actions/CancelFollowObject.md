# CancelFollowObject { #cancel-follow-object }

Used to cancel the goals given to the follow object action server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<CancelFollowObject server_name="follow_object" server_timeout="10"/>
```
