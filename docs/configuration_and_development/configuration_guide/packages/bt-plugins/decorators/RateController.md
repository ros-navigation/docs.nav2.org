# RateController { #rate-controller }

A node that throttles the tick rate for its child.
The tick rate can be supplied to the node as a parameter.
The node returns RUNNING when it is not ticking its child.
Currently, in the navigation stack, the `RateController` is
used to adjust the rate at which the `ComputePathToPose` and `GoalReached` nodes are ticked.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<RateController hz="1.0">
    <!--Add tree components here-->
</RateController>
```
