.. bt_actions:

Spin
====

Invokes the Spin ROS2 action server, which is implemented by the nav2_recoveries module. This action is using in nav2 Behavior Trees as a recovery behavior.

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

  <Spin spin_dist="1.57"/>
    