# ReinitializeGlobalLocalization { #reinitialize-global-localization }

Used to trigger global relocalization using AMCL in case of severe delocalization or kidnapped robot problem.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/reinitialize_global_localization_service.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
