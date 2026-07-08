# IsStuck { #is-stuck }

Determines if the robot is not progressing towards the goal.
If the robot is stuck and not progressing, the condition returns
SUCCESS, otherwise it returns FAILURE.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/condition/is_stuck_condition.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
