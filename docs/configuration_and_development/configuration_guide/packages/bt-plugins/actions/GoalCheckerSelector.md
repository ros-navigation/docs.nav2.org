# GoalCheckerSelector { #goal-checker-selector }

It is used to select the GoalChecker that will be used by the goal_checker server. It subscribes to the `goal_checker_selector` topic to receive command messages with the name of the GoalChecker to be used. It is commonly used before of the FollowPathAction. The `selected_goal_checker` output port is passed to `goal_checker_id` input port of the FollowPathAction. If none is provided on the topic, the `default_goal_checker` is used.

Any publisher to this topic needs to be configured with some QoS defined as `reliable` and `transient local`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/goal_checker_selector_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
