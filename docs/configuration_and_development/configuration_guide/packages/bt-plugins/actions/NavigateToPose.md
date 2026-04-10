# NavigateToPose { #navigate-to-pose }

Invokes the NavigateToPose ROS 2 action server, which is implemented by the [bt_navigator](https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator) module.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<NavigateToPose goal="{goal}" server_name="NavigateToPose" server_timeout="10"
                error_code_id="{navigate_to_pose_error_code}" error_msg="{navigate_to_pose_error_msg}"
                behavior_tree="NavigateThroughPosesWReplanningAndRecovery"/>
```
