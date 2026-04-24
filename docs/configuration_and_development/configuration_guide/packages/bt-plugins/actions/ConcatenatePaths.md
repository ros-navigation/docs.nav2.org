# ConcatenatePaths { #concatenate-paths }

Concatenates two paths into a single path, in order such that the output is `input_path1 + input_path2`.
May be used with multiple of these calls sequentially to concatenate multiple paths.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/concatenate_paths_action.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
