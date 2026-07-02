# ExtractRouteNodesAsGoals { #extract-route-nodes-as-goals }

Concatenates two paths into a single path, in order such that the output is `input_path1 + input_path2`.
May be used with multiple of these calls sequentially to concatenate multiple paths.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/extract_route_nodes_as_goals_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
