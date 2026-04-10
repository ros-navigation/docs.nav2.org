# TruncatePath { #truncate-path }

A custom control node, which modifies a path making it shorter. It removes parts of the path closer than a distance to the goal pose. The resulting last pose of the path orientates the robot to the original goal pose.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<TruncatePath distance="1.0" input_path="{path}" output_path="{truncated_path}"/>
```
