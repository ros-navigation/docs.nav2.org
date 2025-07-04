.. _bt_goal_updated_controller:

GoalUpdatedController
=====================

Checks if the global navigation goal, or a vector of goals, has changed in the blackboard. The node ticks its child if the goal was updated.

Input Ports
-----------

:goal:

  =============================== ========
  Type                            Default
  ------------------------------- --------
  geometry_msgs::msg::PoseStamped "{goal}"
  =============================== ========

  Description
    	Destination to check. Takes in a blackboard variable, "{goal}" if not specified.

:goals:

  ==================== =========
  Type                 Default
  -------------------- ---------
  nav_msgs::msg::Goals "{goals}"
  ==================== =========

  Description
    	Vector of goals to check. Takes in a blackboard variable, "{goals}" if not specified.

Example
-------

.. code-block:: xml

  <GoalUpdatedController goal="{goal}" goals="{goals}">
    <!--Add tree components here--->
  </GoalUpdatedController>
