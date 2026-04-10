# ComputeRoute { #compute-route }

Invokes the ComputeRoute ROS 2 action server, which is implemented by the [nav2_route](https://github.com/ros-navigation/navigation2/tree/main/nav2_route) module.
The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<ComputeRoute start="{start}" goal="{goal}" use_poses="{true}" use_start="{true}" path="{path}" server_name="ComputeRoute" server_timeout="10"
                   error_code_id="{compute_route_error_code}" error_msg="{compute_route_error_msg}"/>
```
