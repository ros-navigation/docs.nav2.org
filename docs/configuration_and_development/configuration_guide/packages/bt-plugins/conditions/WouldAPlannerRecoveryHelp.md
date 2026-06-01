# WouldAPlannerRecoveryHelp { #would-a-planner-recovery-help }

Checks if the active planner server error code is UNKNOWN, NO_VALID_PATH, or TIMEOUT.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/condition/would_a_planner_recovery_help_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
