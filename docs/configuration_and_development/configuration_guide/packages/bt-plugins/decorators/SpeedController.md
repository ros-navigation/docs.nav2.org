# SpeedController { #speed-controller }

A node that controls the tick rate for its child based on current robot speed.
The maximum and minimum replanning rates can be supplied to the node as parameters along with maximum and minimum speed.
The node returns RUNNING when it is not ticking its child. Currently, in the navigation
stack, the `SpeedController` is used to adjust the rate at which the `ComputePathToPose` and `GoalReached` nodes are ticked.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<SpeedController min_rate="0.1" max_rate="1.0" min_speed="0.0" max_speed="0.5" filter_duration="0.3">
  <!--Add tree components here-->
</SpeedController>
```
