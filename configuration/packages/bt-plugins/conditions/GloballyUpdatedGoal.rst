.. _bt_globally_updated_goal_condition:

GloballyUpdatedGoal
===================

Checks if the global navigation goal has changed in the blackboard. 
Returns failure if the goal is the same, if it changes, it returns success.

This node differs from the GoalUpdated by retaining the state of the current goal/goals throughout each tick of the BehaviorTree
such that it will update on any "global" change to the goal. 

Example
-------

.. code-block:: xml

    <GlobalUpdatedGoal/>