# CancelComputeAndTrackRoute { #cancel-compute-and-track-route }

Used to cancel the compute and track route action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<CancelComputeAndTrackRoute server_name="compute_and_track_route" server_timeout="10"/>
```
