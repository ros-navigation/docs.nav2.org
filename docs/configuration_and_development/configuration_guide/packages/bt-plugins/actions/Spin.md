# Spin { #spin }

Invokes the Spin ROS 2 action server, which is implemented by the [nav2_behaviors](https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors) module.
It performs an in-place rotation by a given angle.
This action is used in nav2 Behavior Trees as a recovery behavior.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<Spin spin_dist="1.57" server_name="spin" server_timeout="10" is_recovery="true" disable_collision_checks="false"
      error_code_id="{spin_error_code}" error_msg="{spin_error_msg}"/>
```
