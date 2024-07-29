.. _bt_undock_robot_action:

UndockRobot
===========

Invokes the UndockRobot ROS 2 action server, which is implemented by the docking server.

It is used to undock the robot from a docking station.

Input Ports
***********

:dock_type:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    	The dock plugin type, if not previous instance used for docking.

:max_undocking_time:

  ===== =======
  Type  Default
  ----- -------
  float 30.0
  ===== =======

  Description
    	Maximum time to get back to the staging pose.

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
    	Dock robot error code. See ``UndockRobot`` action message for the enumerated set of error codes.

Example
-------

.. code-block:: xml

  <UndockRobot dock_type="{dock_type}"/>
