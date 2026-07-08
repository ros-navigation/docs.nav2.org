# WouldARouteRecoveryHelp { #would-a-route-recovery-help }

Checks if the active route server error code is UNKNOWN, NO_VALID_ROUTE, or TIMEOUT.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/condition/would_a_route_recovery_help_condition.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
