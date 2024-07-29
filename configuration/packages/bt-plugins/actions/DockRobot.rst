.. _bt_dock_robot_action:

DockRobot
=========

Invokes the DockRobot ROS 2 action server, which is implemented by the docking server.

It is used to dock the robot to a docking station.

Input Ports
***********

:use_dock_id:

  ==== =======
  Type Default
  ---- -------
  bool true
  ==== =======

  Description
    	Whether to use the dock's ID or dock pose fields.

:dock_id:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    	Dock ID or name to use.

:dock_pose:

  ========================= =======
  Type                      Default
  ------------------------- -------
  geometry_msgs/PoseStamped N/A
  ========================= =======

  Description
    	The dock pose, if not using dock id.

:dock_type:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    	The dock plugin type, if using dock pose.

:max_staging_time:

  ===== =======
  Type  Default
  ----- -------
  float 1000.0
  ===== =======

  Description
    	Maximum time to navigate to the staging pose.

:navigate_to_staging_pose:

  ==== =======
  Type Default
  ---- -------
  bool true
  ==== =======

  Description
    	Whether to autonomously navigate to staging pose.

Output Ports
------------

:success:

  ==== =======
  Type Default
  ---- -------
  bool true
  ==== =======

  Description
    	If the action was successful.

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          0  
  ============== =======

  Description
    	Dock robot error code. See ``DockRobot`` action message for the enumerated set of error codes.

:num_retries:

  ====== =======
  Type   Default
  ------ -------
  uint16 0
  ====== =======

  Description
    	The number of retries executed.

Example
-------

.. code-block:: xml

  <DockRobot dock_id="{dock_id}" error_code_id="{dock_error_code}"/>
