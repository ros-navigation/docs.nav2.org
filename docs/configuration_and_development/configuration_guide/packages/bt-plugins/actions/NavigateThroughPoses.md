# NavigateThroughPoses { #navigate-through-poses }

Invokes the NavigateThroughPoses ROS 2 action server, which is implemented by the [bt_navigator](https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator) module.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<NavigateThroughPoses goals="{goals}" server_name="NavigateThroughPoses" server_timeout="10"
                      error_code_id="{navigate_through_poses_error_code}" error_msg="{navigate_through_poses_error_msg}"
                      behavior_tree="<some-path>/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"/>
```
