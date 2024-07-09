.. _bt_getposefrompath_action:

GetPoseFromPath
===============

Gets a pose from a particular index on the path. Use ``-1`` to get the last pose, ``-2`` for second to last, and so on.

Input Ports
***********

:path:

  ============= =======
  Type          Default
  ------------- -------
  nav_msgs/Path N/A
  ============= =======

  Description
    	Path to extract pose from

:index:

  ====== =======
  Type   Default
  ------ -------
  int    0
  ====== =======

  Description
    	Index from path to use. Use ``-1`` to get the last pose, ``-2`` for second to last, and so on.

Output Ports
------------

:pose:

  ========================= =======
  Type                      Default
  ------------------------- -------
  geometry_msgs/PoseStamped N/A  
  ========================= =======

  Description
    	Pose from path, with the Path's set header.

Example
-------

.. code-block:: xml

  <GetPoseFromPath path="{path}" index="-1" pose="{goal}"/>
