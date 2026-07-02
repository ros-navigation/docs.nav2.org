# ControllerSelector { #controller-selector }

It is used to select the Controller that will be used by the Controller server. It subscribes to the `controller_selector` topic to receive command messages with the name of the Controller to be used. It is commonly used before of the FollowPathAction. The `selected_controller` output port is passed to `controller_id` input port of the FollowPathAction. If none is provided on the topic, the `default_controller` is used.

Any publisher to this topic needs to be configured with some QoS defined as `reliable` and `transient local`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/controller_selector_node.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
