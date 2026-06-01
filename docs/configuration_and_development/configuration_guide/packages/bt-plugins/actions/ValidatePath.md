# ValidatePath { #validate-path }

Checks to see if the global path is valid. If there is an
obstacle along the path, it returns FAILURE, otherwise
it returns SUCCESS. Optionally checks specific costmap layers and
can use a custom footprint for validation.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/action/validate_path_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
