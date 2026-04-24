# ProgressCheckerSelector { #progress-checker-selector }

It is used to select the ProgressChecker that will be used by the progress_checker server. It subscribes to the `progress_checker_selector` topic to receive command messages with the name of the ProgressChecker to be used. It is commonly used before of the FollowPathAction. The `selected_progress_checker` output port is passed to `progress_checker_id` input port of the FollowPathAction. If none is provided on the topic, the `default_progress_checker` is used.

Any publisher to this topic needs to be configured with some QoS defined as `reliable` and `transient local`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/progress_checker_selector_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
