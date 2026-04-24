# CancelComputeAndTrackRoute { #cancel-compute-and-track-route }

Used to cancel the compute and track route action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/action/compute_and_track_route_cancel_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
