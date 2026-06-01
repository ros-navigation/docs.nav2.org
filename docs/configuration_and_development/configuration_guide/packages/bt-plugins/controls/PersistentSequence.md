# PersistentSequence { #persistent-sequence }

The PersistentSequenceNode is similar to the SequenceNode, but it stores the index of the last running child in the blackboard (key: *current_child_idx*), and it does not reset the index when it got halted. It used to tick children in an ordered sequence. If any child returns RUNNING, previous children will NOT be ticked again.
This can be helpful paired with the `PauseResumeController`.

- If all the children return SUCCESS, this node returns SUCCESS.
- If a child returns RUNNING, this node returns RUNNING. Loop is NOT restarted, the same running child will be ticked again.
- If a child returns FAILURE, stop the loop and return FAILURE. Restart the loop only if (reset_on_failure == true)

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/control/persistent_sequence.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
