# WouldARouteRecoveryHelp { #would-a-route-recovery-help }

Checks if the active route server error code is UNKNOWN, NO_VALID_ROUTE, or TIMEOUT.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/condition/would_a_route_recovery_help_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
