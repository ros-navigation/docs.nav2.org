# RecoveryNode { #recovery-node }

The RecoveryNode is a control flow node with two children.
It returns SUCCESS if and only if the first child returns SUCCESS.
The second child will be executed only if the first child returns FAILURE.
If the second child SUCCEEDS, then the first child will be executed again.
If the second child returns FAILURE, the RecoveryNode returns FAILURE as well.
The user can specify how many times the recovery actions should be taken before returning FAILURE.
In nav2, the RecoveryNode is included in Behavior Trees to implement recovery actions upon failures.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/control/recovery_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
