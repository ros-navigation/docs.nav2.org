# WouldAPlannerRecoveryHelp { #would-a-planner-recovery-help }

Checks if the active planner server error code is UNKNOWN, NO_VALID_PATH, or TIMEOUT.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<WouldAPlannerRecoveryHelp error_code="{compute_path_to_pose_error_code}"/>
```
