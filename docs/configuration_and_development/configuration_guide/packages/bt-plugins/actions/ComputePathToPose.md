# ComputePathToPose { #compute-path-to-pose }

Invokes the ComputePathToPose ROS 2 action server, which is implemented by the [nav2_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_planner) module.
The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" server_name="ComputePathToPose" server_timeout="10"
                   error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>
```
