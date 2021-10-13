.. _bt_spin_action:

Spin
====

Invokes the Spin ROS 2 action server, which is implemented by the nav2_recoveries_ module.
It performs an in-place rotation by a given angle. 
This action is used in nav2 Behavior Trees as a recovery behavior.

.. _nav2_recoveries: https://github.com/ros-planning/navigation2/tree/main/nav2_recoveries

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
      Time to envoke recovery for, if exceeds considers it a stuck condition or failure case (seconds).

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

Example
-------

.. code-block:: xml

  <Spin spin_dist="1.57" server_name="spin" server_timeout="10"/>
    
