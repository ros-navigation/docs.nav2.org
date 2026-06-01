# PathHandlerSelector { #path-handler-selector }

It is used to select the PathHandler that will be used by the controller server. It subscribes to the `path_handler_selector` topic to receive command messages with the name of the PathHandler to be used. It is commonly used before of the FollowPathAction. The `selected_path_handler` output port is passed to `path_handler_id` input port of the FollowPathAction. If none is provided on the topic, the `default_path_handler` is used.

Any publisher to this topic needs to be configured with some QoS defined as `reliable` and `transient local`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/path_handler_selector_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
