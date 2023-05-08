.. _bt_spin_action:

Spin
====

Invokes the Spin ROS 2 action server, which is implemented by the nav2_behaviors_ module.
It performs an in-place rotation by a given angle.
This action is used in nav2 Behavior Trees as a recovery behavior.

.. _nav2_behaviors: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors

Input Ports
-----------

:spin_dist:

  ====== =======
  Type   Default
  ------ -------
  double 1.57
  ====== =======

  Description
    	Spin distance (radians).

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

:is_recovery:

  ==== =======
  Type Default
  ---- -------
  bool True
  ==== =======

  Description
    	True if the action is being used as a recovery.

Output Ports
------------

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A  
  ============== =======

  Description
    	Spin error code. See ``Spin`` action message for the enumerated set of error codes.

Example
-------

.. code-block:: xml

  <Spin spin_dist="1.57" server_name="spin" server_timeout="10" is_recovery="true" error_code_id="{spin_error_code}"/>

