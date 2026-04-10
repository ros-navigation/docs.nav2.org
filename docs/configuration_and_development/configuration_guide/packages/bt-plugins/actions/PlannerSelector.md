# PlannerSelector { #planner-selector }

It is used to select the planner that will be used by the planner server. It subscribes to the `planner_selector` topic to receive command messages with the name of the planner to be used. It is commonly used before of the ComputePathToPoseAction. The `selected_planner` output port is passed to `planner_id` input port of the ComputePathToPoseAction. If none is provided on the topic, the `default_planner` is used.

Any publisher to this topic needs to be configured with some QoS defined as `reliable` and `transient local`.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
```
