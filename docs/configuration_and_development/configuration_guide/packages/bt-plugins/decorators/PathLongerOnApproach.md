# PathLongerOnApproach { #path-longer-on-approach }

This node checks if the newly generated global path is significantly larger than the old global path in the user-defined robot’s goal proximity and triggers their corresponding children. This allows users to enact special behaviors before a robot attempts to execute a path significantly longer than the prior path when close to a goal (e.g. going around an dynamic obstacle that may just need a few seconds to move out of the way).

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<PathLongerOnApproach path="{path}" prox_len="3.0" length_factor="2.0">
  <!--Add tree components here-->
</PathLongerOnApproach>
```
