# IsPathValid { #is-path-valid }

Checks to see if the global path is valid. If there is an
obstacle along the path, it returns FAILURE, otherwise
it returns SUCCESS. Optionally checks specific costmap layers and
can use a custom footprint for validation.

<!-- 
In Rolling, the IsPathValid node has been renamed to ValidatePath
Use the new name for generation as BT XML has not yet been updated

TODO: replace with render_bt_node_ports(page.title) after backporting the BT XML file to Jazzy  
-->
{{ render_bt_node_ports("ValidatePath") }}

## Example

<!-- 
TODO:replace with 
{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/condition/is_path_valid_condition.hpp" %} 
after backporting usage examples to Jazzy
-->
{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/validate_path_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
