# ComputeCoveragePath { #compute-coverage-path }

Invokes the ComputeCoveragePath ROS 2 action server, which is implemented by the [opennav_coverage](https://github.com/open-navigation/opennav_coverage) server module.
The server address can be remapped using the `server_name` input port.
This server can take in both cartesian and GPS coordinates and is implemented using the `Fields2Cover` library.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = opennav_cov_bt_plugins_hpp_path + "/compute_complete_coverage_path.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}

Note: the blackboard IDs for the path, error code, and more may be adjusted, but need to match the corresponding parameters in the `CoverageNavigator` plugin to set on the blackboard for use from the action server.
