# GetNextFewGoals { #get-next-few-goals }

Extracts only the next `N` goals from a list of goals to send to a later task that only needs localized future knowledge.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/get_next_few_goals_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
