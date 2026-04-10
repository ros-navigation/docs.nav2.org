# CancelDriveOnHeading { #cancel-drive-on-heading }

Used to cancel the drive on heading action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<CancelDriveOnHeading server_name="drive_on_heading" server_timeout="10"/>
```
