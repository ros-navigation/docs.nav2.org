.. _bt_assisted_teleop_action:

AssistedTeleop
==============

Invokes the AssistedTeleop ROS 2 action server, which filters teleop twist commands to prevent
collisions. This is used in nav2 Behavior Trees as a recovery behavior or a regular behavior.
The nav2_behaviors_ module implements the AssistedTeleop action server.

.. _nav2_behaviors: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors


Input Ports
***********

:is_recovery:

  ====== =======
  Type   Default
  ------ -------
  double false
  ====== =======

  Description
      If true increment the recovery counter.

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

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A  
  ============== =======

  Description
    	Assisted teleop error code. See ``AssistedTeleop`` action message for the enumerated set of error codes.

Example
-------

.. code-block:: xml

  <AssistedTeleop is_recovery="false" server_name="assisted_teleop_server" server_timeout="10" error_code_id="{assisted_teleop_error_code}"/>
