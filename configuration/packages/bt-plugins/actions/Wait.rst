.. bt_actions:

Wait
====

Invokes the Wait ROS2 action server, which is implemented by the nav2_recoveries module. 
This action is used in nav2 Behavior Trees as a recovery behavior.


Input Ports
-----------

:wait_duration:

  ====== =======
  Type   Default
  ------ -------
  double 1.0
  ====== =======

  Description
    	Wait time (s).

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

  <Wait wait_duration="1.0" server_name="wait_server" server_timeout="10"/>
