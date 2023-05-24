.. _bt_backup_action:

BackUp
======

Invokes the BackUp ROS 2 action server, which causes the robot to back up by a specific displacement.
It performs an linear translation by a given distance.
This is used in nav2 Behavior Trees as a recovery behavior. The nav2_behaviors module implements the BackUp action server.

.. nav2_behaviors_: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors

Input Ports
***********

:backup_dist:

  ====== =======
  Type   Default
  ------ -------
  double -0.15
  ====== =======

  Description
    	Total distance to backup (m).

:backup_speed:

  ====== =======
  Type   Default
  ------ -------
  double 0.025
  ====== =======

  Description
    	Backup speed (m/s).

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
    	Backup error code. See ``BackUp`` action message for the enumerated set of error codes.

Example
-------

.. code-block:: xml

  <BackUp backup_dist="-0.2" backup_speed="0.05" server_name="backup_server" server_timeout="10" error_code_id="{backup_error_code}"/>
