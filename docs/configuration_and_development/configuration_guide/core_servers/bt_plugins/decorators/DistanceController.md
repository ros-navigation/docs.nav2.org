# DistanceController { #distance-controller }

A node that controls the tick rate for its child based on the distance traveled.
The distance to be traveled before replanning can be supplied to the node as a parameter.
The node returns RUNNING when it is not ticking its child. Currently, in the navigation
stack, the `DistanceController` is used to adjust the rate at which the `ComputePathToPose` and `GoalReached` nodes are ticked.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/decorator/distance_controller.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
