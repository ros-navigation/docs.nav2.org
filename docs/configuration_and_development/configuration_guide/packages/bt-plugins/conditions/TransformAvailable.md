# TransformAvailable { #transform-available }

Checks if a TF transform is available. Returns failure if it cannot be found. Once found, it will always return success. Useful for initial condition checks.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<TransformAvailable parent="odom" child="base_link"/>
```
