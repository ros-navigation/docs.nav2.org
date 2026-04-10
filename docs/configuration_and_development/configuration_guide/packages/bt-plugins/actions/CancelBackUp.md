# CancelBackUp { #cancel-back-up }

Used to cancel the backup action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<CancelBackUp server_name="BackUp" server_timeout="10"/>
```
