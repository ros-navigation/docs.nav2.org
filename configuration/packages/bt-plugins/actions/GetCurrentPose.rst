.. _bt_get_current_pose_action:

GetCurrentPose
==============

Obtains the current pose from TF and places it on the blackboard for other nodes to use.

Input Ports
-----------

:robot_base_frame:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  string                          N/A
  =============================== =======

  Description
        Robot base frame to transform poses to if not given in the same frame. If not provided, uses the BT Navigator's ``base_frame`` setting automatically.

:global_frame:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
        Global frame to transform poses to if not given in the same frame. If not provided, uses the BT Navigator's ``global_frame`` setting automatically.

Output Ports
------------

:current_pose:

  =============================== =======
  Type                           Default
  ------------------------------- -------
  geometry_msgs::msg::PoseStamped N/A
  =============================== =======

  Description
    	The current pose in the global frame.

Example
-------

.. code-block:: xml

    <GetCurrentPose current_pose="{current_pose}"/>
