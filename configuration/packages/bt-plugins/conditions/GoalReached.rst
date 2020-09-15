.. _bt_goal_reached_condition:

GoalReached
===========

Checks the distance to the goal, if the distance to goal is less than the pre-defined threshold, the tree returns SUCCESS, otherwise it returns FAILURE.


Parameter
---------

:transform_tolerance:

  Defined and declared in :ref:`configuring_bt_navigator`.

:goal_reached_tol:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======

  Description
    	Tolerance of accepting pose as the goal (m).

Example
^^^^^^^
.. code-block:: yaml

    bt_navigator:
      ros__parameters:
        # other bt_navigator parameters
        transform_tolerance: 0.1
        goal_reached_tol: 0.25


Input Ports
-----------

:goal:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    	Destination to check. Takes in a blackboard variable, e.g. "{goal}".

:global_frame:

  ====== =======
  Type   Default
  ------ -------
  string "map"
  ====== =======

  Description
    	Reference frame.

:robot_base_frame:

  ====== ===========
  Type   Default
  ------ -----------
  string "base_link"
  ====== ===========

  Description
    	Robot base frame.

Example
^^^^^^^

.. code-block:: xml

  <GoalReached goal="{goal}" global_frame="map" robot_base_frame="base_link"/>
