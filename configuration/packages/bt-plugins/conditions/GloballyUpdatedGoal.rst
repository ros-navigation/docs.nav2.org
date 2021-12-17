.. _bt_globally_updated_goal_condition:

GloballyUpdatedGoal
===================

hecks if the global navigation goal has changed in the blackboard. 
Returns failure if the goal is the same, if it changes, it returns success.

This node differs from the GoalUpdated condition by checking if the goal has updated within the context of the entire BT

Example
-------

.. code-block:: xml

    <GlobalUpdatedGoal/>