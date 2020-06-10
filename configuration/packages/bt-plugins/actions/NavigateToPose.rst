.. bt_actions:

NavigateToPose
==============

Invokes the NavigateToPose ROS2 action server, which is implemented by the bt_navigator_ module.

.. _bt_navigator: https://github.com/ros-planning/navigation2/tree/master/nav2_bt_navigator

Input Ports
-----------

:position:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	Position.

:orientation:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	Orientation.

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

  <NavigateToPose position="???" orientation="???" server_name="NavigateToPose_server" server_timeout="10"/>
