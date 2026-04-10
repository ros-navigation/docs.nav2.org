# DriveOnHeading { #drive-on-heading }

Invokes the DriveOnHeading ROS 2 action server, which causes the robot to drive on the current heading by a specific displacement.
It performs a linear translation by a given distance. The nav2_behaviors module implements the DriveOnHeading action server.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<DriveOnHeading dist_to_travel="0.2" speed="0.05" server_name="backup_server" server_timeout="10" disable_collision_checks="false"
                error_code_id="{drive_on_heading_error_code}" error_msg="{drive_on_heading_error_msg}"/>
```
