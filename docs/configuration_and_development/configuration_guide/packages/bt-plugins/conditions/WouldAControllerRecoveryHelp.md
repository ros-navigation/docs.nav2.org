# WouldAControllerRecoveryHelp { #would-a-controller-recovery-help }

Checks if the active controller server error code is UNKNOWN, PATIENCE_EXCEEDED, FAILED_TO_MAKE_PROGRESS, or NO_VALID_CONTROL.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
```
