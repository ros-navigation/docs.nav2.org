.. _goal_updated_condition:

GoalUpdated
===========

Checks if the global navigation goal, or a vector of goals, has changed in the blackboard. 
Returns failure if the goal is the same, if it changes, it returns success.

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

  ==================================== =========
  Type                                 Default
  ------------------------------------ ---------
  geometry_msgs::msg::PoseStampedArray "{goals}"
  ==================================== =========

  Description
    	Vector of goals to check. Takes in a blackboard variable, "{goals}" if not specified.

Example
-------

.. code-block:: xml

    <GoalUpdated/>
