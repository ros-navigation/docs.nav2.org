.. bt_conditions:

GoalReached
===========

Checks the distance to the goal, if the distance to goal is less than the pre-defined threshold, the tree returns SUCCESS, otherwise it returns FAILURE.

Input Ports
-----------

:goal:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    	Destination to check.

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
-------

.. code-block:: xml

  <GoalReached goal="{goal}" global_frame="map" robot_base_frame="base_link"/>
