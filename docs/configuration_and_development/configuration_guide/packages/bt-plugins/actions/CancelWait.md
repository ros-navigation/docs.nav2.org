# CancelWait { #cancel-wait }

Used to cancel the wait action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<CancelWait server_name="Wait" server_timeout="10"/>
```
