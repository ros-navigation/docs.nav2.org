# PlannerSelector { #planner-selector }

It is used to select the planner that will be used by the planner server. It subscribes to the `planner_selector` topic to receive command messages with the name of the planner to be used. It is commonly used before of the ComputePathToPoseAction. The `selected_planner` output port is passed to `planner_id` input port of the ComputePathToPoseAction. If none is provided on the topic, the `default_planner` is used.

Any publisher to this topic needs to be configured with some QoS defined as `reliable` and `transient local`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/planner_selector_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
