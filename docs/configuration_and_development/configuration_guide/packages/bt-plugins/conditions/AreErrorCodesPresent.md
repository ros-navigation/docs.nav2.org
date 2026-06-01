# AreErrorCodesPresent { #are-error-codes-present }

Checks the if the provided error code matches any error code within a set.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/condition/are_error_codes_present_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
