# FollowObject { #follow-object }

Invokes the FollowObject ROS 2 action server, it will dynamically follow an object while maintaining a defined distance.
The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<FollowObject name="FollowPerson" pose_topic="/person_pose" max_duration="0.0"/>
```
