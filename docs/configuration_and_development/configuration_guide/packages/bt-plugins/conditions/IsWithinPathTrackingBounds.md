# IsWithinPathTrackingBounds { #is-within-path-tracking-bounds }

Checks if the robot is within determined tracking error bounds during path following.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<IsWithinPathTrackingBounds max_error_left="0.5" max_error_right="0.5" max_error_heading="3.14" tracking_feedback="{tracking_feedback}"/>
```
