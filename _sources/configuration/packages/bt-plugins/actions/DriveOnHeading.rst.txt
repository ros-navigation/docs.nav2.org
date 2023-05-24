.. _bt_driveonheading_action:

DriveOnHeading
==============

Invokes the DriveOnHeading ROS 2 action server, which causes the robot to drive on the current heading by a specific displacement.
It performs a linear translation by a given distance. The nav2_behaviors module implements the DriveOnHeading action server.

.. nav2_behaviors_: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors

Input Ports
***********

:dist_to_travel:

  ====== =======
  Type   Default
  ------ -------
  double 0.15
  ====== =======

  Description
    	Distance to travel (m).

:speed:

  ====== =======
  Type   Default
  ------ -------
  double 0.025
  ====== =======

  Description
    	Speed at which to travel (m/s).

:time_allowance:

  ====== =======
  Type   Default
  ------ -------
  double 10.0
  ====== =======

  Description
      Time to envoke behavior for, if exceeds considers it a stuck condition or failure case (seconds).

:server_name:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    	Action server name.

:server_timeout:

  ====== =======
  Type   Default
  ------ -------
  double 10
  ====== =======

  Description
    	Action server timeout (ms).

Output Ports
------------

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A  
  ============== =======

  Description
    	Drive on heading error code. See ``DriveOnHeading`` action message for the enumerated set of error codes.

Example
-------

.. code-block:: xml

  <DriveOnHeading dist_to_travel="0.2" speed="0.05" server_name="backup_server" server_timeout="10" error_code_id="{drive_on_heading_error_code}"/>
