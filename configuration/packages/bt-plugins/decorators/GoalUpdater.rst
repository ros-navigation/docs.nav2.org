.. _bt_goal_updater:

GoalUpdater
===========

A custom control node, which updates the goal(s) pose(s). It subscribes to a topic in which it can receive (an) updated goal(s) pose(s) to use instead of the one(s) commanded in action. It is useful for dynamic object following tasks.

Parameters
----------

:goal_updater_topic:

  ====== ==============
  Type   Default
  ------ --------------
  string  "goal_update"
  ====== ==============

  Description
      The topic to receive the updated goal pose

:goals_updater_topic:

  ====== ===============
  Type   Default
  ------ ---------------
  string  "goals_update"
  ====== ===============

  Description
      The topic to receive the updated goals poses

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

:input_goals:

  ============================== =======
  Type                           Default
  ------------------------------ -------
  geometry_msgs/PoseStampedArray   N/A
  ============================== =======

  Description
      The original goals poses

:output_goal:

  ========================= =======
  Type                      Default
  ------------------------- -------
  geometry_msgs/PoseStamped N/A
  ========================= =======

  Description
    	The resulting updated goal. If no goal received by subscription, it will be the input_goal

:output_goals:
  
    ============================== =======
    Type                           Default
    ------------------------------ -------
    geometry_msgs/PoseStampedArray   N/A
    ============================== =======
  
    Description
      	The resulting updated goals. If no goals received by subscription, it will be the input_goals

Example
-------

.. code-block:: xml

  <GoalUpdater input_goal="{goal}" input_goals="{goals}" output_goal="{goal}" output_goals="{goals}">
    <!--Add tree components here--->
  </GoalUpdater>
