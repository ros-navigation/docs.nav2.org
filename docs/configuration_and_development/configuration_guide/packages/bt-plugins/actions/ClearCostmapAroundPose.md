# ClearCostmapAroundPose { #clear-costmap-around-pose }

Action to call a costmap clearing around a given pose server.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<ClearCostmapAroundPose name="ClearLocalCostmapAroundPose"
                        service_name="local_costmap/clear_around_pose_local_costmap"
                        pose="{goal_pose}"
                        reset_distance="2.0"/>
```
