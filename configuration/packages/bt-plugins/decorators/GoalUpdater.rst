.. _bt_goal_updater:

GoalUpdater
===========

A custom control node, which updates the goal pose. It subscribes to a topic in which it can receive an updated goal pose to use instead of the one commanded in action. It is useful for dynamic object following tasks.

Parameters
----------

:change_goal_topic:

  ====== ==============
  Type   Default
  ------ --------------
  string  "goal_update"
  ====== ==============

  Description
      The topic to receive the updated goal pose

Input Ports
-----------

:input_goal:

  ========================= =======
  Type                      Default
  ------------------------- -------
  geometry_msgs/PoseStamped N/A
  ========================= =======

  Description
      The original goal pose

:output_goal:

  ========================= =======
  Type                      Default
  ------------------------- -------
  geometry_msgs/PoseStamped N/A
  ========================= =======

  Description
    	The resulting updated goal. If no goal received by subscription, it will be the input_goal

Example
-------

.. code-block:: xml

  <ChangeGoal input_goal="{goal}" output_goal="{updated_goal}">
    <!--Add tree components here--->
  </ChangeGoal>
