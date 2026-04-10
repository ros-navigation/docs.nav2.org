# WouldASmootherRecoveryHelp { #would-a-smoother-recovery-help }

Checks if the active smoother server error code is UNKNOWN, TIMEOUT, FAILED_TO_SMOOTH_PATH, or SMOOTHED_PATH_IN_COLLISION.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<WouldASmootherRecoveryHelp error_code="{smoother_error_code}"/>
```
