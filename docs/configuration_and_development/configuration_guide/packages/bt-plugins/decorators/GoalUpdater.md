# GoalUpdater { #goal-updater }

A custom control node, which updates the goal(s) pose(s). It subscribes to a topic in which it can receive (an) updated goal(s) pose(s) to use instead of the one(s) commanded in action. It is useful for dynamic object following tasks.

## Parameters

### **`goal_updater_topic`**

| Type     | Default       |
|----------|---------------|
| `string` | "goal_update" |

Description
:   The topic to receive the updated goal pose

### **`goals_updater_topic`**

| Type     | Default        |
|----------|----------------|
| `string` | "goals_update" |

Description
:   The topic to receive the updated goals poses

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_bt_plugins_hpp_path + "/decorator/goal_updater_node.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
