.. _bt_are_poses_near_action:

ArePosesNear
============

Checks if two poses are nearby. If the input poses are in different frames, it will automatically transform both to the global frame.

Input Ports
-----------

:ref_pose:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  geometry_msgs::msg::PoseStamped N/A
  =============================== =======

  Description
        Takes in a blackboard variable containing the initial pose to check.

:target_pose:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  geometry_msgs::msg::PoseStamped N/A
  =============================== =======

  Description
        Takes in a blackboard variable containing the other pose to check against.

:global_frame:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
        Global frame to transform poses to if not given in the same frame. If not provided, uses the BT Navigator's ``global_frame`` setting automatically.

:tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.50
  ====== =======

  Description
      Tolerance to check poses if nearby with respect to.

Example
-------

.. code-block:: xml

    <ArePosesNear ref_pose="{init_pose}" target_pose="{goal_pose}" tolerance="0.10"/>
