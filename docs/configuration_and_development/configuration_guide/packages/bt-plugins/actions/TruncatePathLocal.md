# TruncatePathLocal { #truncate-path-local }

A custom control node, which modifies a path making it shorter. It removes parts of the path which are more distant than specified forward/backward distance around robot

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<TruncatePathLocal input_path="{path}" output_path="{path_local}" distance_forward="3.5" distance_backward="2.0" robot_frame="base_link"/>
```
