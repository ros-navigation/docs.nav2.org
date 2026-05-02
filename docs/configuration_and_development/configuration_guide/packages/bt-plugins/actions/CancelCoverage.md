# CancelCoverage { #cancel-coverage }

Used to cancel the goals given to the complete coverage action server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + opennav_cov_bt_plugins_hpp_path + "/cancel_complete_coverage_path.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
