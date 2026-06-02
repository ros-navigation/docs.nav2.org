# GoalUpdater { #goal-updater }

A custom control node, which updates the goal pose. It subscribes to a topic in which it can receive an updated goal pose to use instead of the one commanded in action. It is useful for dynamic object following tasks.

## Parameters

### **`goal_updater_topic`**

| Type     | Default       |
|----------|---------------|
| `string` | "goal_update" |

Description
:   The topic to receive the updated goal pose


{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/decorator/goal_updater_node.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
