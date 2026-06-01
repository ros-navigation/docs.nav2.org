# PathLongerOnApproach { #path-longer-on-approach }

This node checks if the newly generated global path is significantly larger than the old global path in the user-defined robot's goal proximity and triggers their corresponding children. This allows users to enact special behaviors before a robot attempts to execute a path significantly longer than the prior path when close to a goal (e.g. going around an dynamic obstacle that may just need a few seconds to move out of the way).

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/decorator/path_longer_on_approach.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
