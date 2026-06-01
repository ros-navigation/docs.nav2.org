# GlobalUpdatedGoal { #global-updated-goal }

Checks if the global navigation goal has changed in the blackboard.
Returns failure if the goal is the same, if it changes, it returns success.

This node differs from the GoalUpdated by retaining the state of the current goal/goals throughout each tick of the BehaviorTree
such that it will update on any "global" change to the goal.

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_plugin_hpp_path = nav2_bt_plugins_hpp_path + "/condition/globally_updated_goal_condition.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
