# FollowPath { #follow-path }

Invokes the FollowPath ROS 2 action server, which is implemented by the controller plugin modules loaded.
The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="precise_goal_checker" path_handler_id="PathHandler" server_name="FollowPath" server_timeout="10" error_code_id="{follow_path_error_code}" error_msg="{follow_path_error_msg}" tracking_feedback="{tracking_feedback}"/>
```
