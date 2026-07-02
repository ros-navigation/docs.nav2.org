# WouldAControllerRecoveryHelp { #would-a-controller-recovery-help }

Checks if the active controller server error code is UNKNOWN, PATIENCE_EXCEEDED, FAILED_TO_MAKE_PROGRESS, or NO_VALID_CONTROL.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/condition/would_a_controller_recovery_help_condition.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
