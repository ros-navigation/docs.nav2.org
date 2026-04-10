# DistanceController { #distance-controller }

A node that controls the tick rate for its child based on the distance traveled.
The distance to be traveled before replanning can be supplied to the node as a parameter.
The node returns RUNNING when it is not ticking its child. Currently, in the navigation
stack, the `DistanceController` is used to adjust the rate at which the `ComputePathToPose` and `GoalReached` nodes are ticked.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<DistanceController distance="0.5" global_frame="map" robot_base_frame="base_link">
  <!--Add tree components here-->
</DistanceController>
```
