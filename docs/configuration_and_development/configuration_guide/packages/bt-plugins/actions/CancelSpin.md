# CancelSpin { #cancel-spin }

Used to cancel the spin action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<CancelSpin server_name="Spin" server_timeout="10"/>
```
