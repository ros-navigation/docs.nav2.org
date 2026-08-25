.. _bt_is_goal_nearby_condition:

IsGoalNearby
============

Checks if the robot is near the goal by computing the remaining path length from the robot's current position to the goal. Returns SUCCESS when the remaining path length is less than the proximity threshold, otherwise returns FAILURE.

Parameter
---------

:transform_tolerance:

  Defined and declared in :ref:`configuring_bt_navigator`.

Example
^^^^^^^
.. code-block:: yaml

    bt_navigator:
      ros__parameters:
        # other bt_navigator parameters
        transform_tolerance: 0.1

Input Ports
-----------

:path:

  ==================================== =======
  Type                                 Default
  ------------------------------------ -------
  nav_msgs::msg::Path                  N/A
  ==================================== =======

  Description
    The planned path to evaluate.

:proximity_threshold:

  ====== =======
  Type   Default
  ------ -------
  double 1.0
  ====== =======

  Description
    The remaining path length (in meters) considered as "nearby". When the remaining distance along the path is less than this threshold, the condition returns SUCCESS.

:max_robot_pose_search_dist:

  ====== =======
  Type   Default
  ------ -------
  double -1.0
  ====== =======

  Description
    Maximum forward integrated distance along the path (starting from the last detected pose) to bound the search for the closest pose to the robot. When set to a negative value (default), the entire path is searched every time. Setting this to a positive value (e.g., 1.0-2.0 meters) can improve performance when this BT node is ticked frequently to address looping or crossed paths (when present).

:global_frame:

  ====== =======
  Type   Default
  ------ -------
  string "map"
  ====== =======

  Description
    The global reference frame.

:robot_base_frame:

  ====== =======
  Type   Default
  ------ -------
  string "base_link"
  ====== =======

  Description
    Robot base frame.

Example
^^^^^^^

.. code-block:: xml

  <IsGoalNearby path="{path}" proximity_threshold="1.0" />
