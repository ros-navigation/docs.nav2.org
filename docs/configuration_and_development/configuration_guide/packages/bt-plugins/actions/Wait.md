# Wait { #wait }

Invokes the Wait ROS 2 action server, which is implemented by the [nav2_behaviors](https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors) module.
This action is used in nav2 Behavior Trees as a recovery behavior.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<Wait wait_duration="1.0" server_name="wait_server" server_timeout="10"
      error_code_id="{wait_error_code}" error_msg="{wait_error_msg}"/>
```
