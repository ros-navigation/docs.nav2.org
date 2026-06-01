# SmootherSelector { #smoother-selector }

It is used to select the Smoother that will be used by the Smoother server. It subscribes to the `smoother_selector` topic to receive command messages with the name of the Smoother to be used. It is commonly used before of the FollowPathAction. If none is provided on the topic, the `default_smoother` is used.

Any publisher to this topic needs to be configured with some QoS defined as `reliable` and `transient local`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/smoother_selector_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
