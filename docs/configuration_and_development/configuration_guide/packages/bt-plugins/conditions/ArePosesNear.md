# ArePosesNear { #are-poses-near }

Checks if two poses are nearby. If the input poses are in different frames, it will automatically transform both to the global frame.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<ArePosesNear ref_pose="{init_pose}" target_pose="{goal_pose}" tolerance="0.10"/>
```
