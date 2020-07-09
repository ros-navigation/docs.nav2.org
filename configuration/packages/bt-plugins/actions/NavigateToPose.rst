.. _bt_navigate_to_pose_action:

NavigateToPose
==============

Invokes the NavigateToPose ROS 2 action server, which is implemented by the bt_navigator_ module.

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
    	Takes in a blackboard variable containing the position, eg. "{position}".

:orientation:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	Takes in a blackboard variable containing the orientation, eg. "{orientation}".

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

  <NavigateToPose position="{position}" orientation="{orientation}" server_name="NavigateToPose" server_timeout="10"/>
