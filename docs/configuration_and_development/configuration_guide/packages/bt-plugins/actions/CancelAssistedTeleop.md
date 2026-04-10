# CancelAssistedTeleop { #cancel-assisted-teleop }

Used to cancel the AssistedTeleop action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<CancelAssistedTeleop server_name="assisted_teleop" server_timeout="10"/>
```
