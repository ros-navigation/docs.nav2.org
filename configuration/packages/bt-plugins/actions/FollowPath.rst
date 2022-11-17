.. _bt_follow_path_action:

FollowPath
==========

Invokes the FollowPath ROS 2 action server, which is implemented by the controller plugin modules loaded. 
The server address can be remapped using the ``server_name`` input port.

Input Ports
-----------

:path:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	Takes in a blackboard variable containing the path to follow, eg. "{path}".

:controller_id:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	Mapped name of the controller plugin type to use, e.g. FollowPath.

:goal_checker_id:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	Mapped name of the goal checker plugin type to use, e.g. SimpleGoalChecker.

:server_name:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	Action server name.


:server_timeout:

  ============== =======
  Type           Default
  -------------- -------
  double         10  
  ============== =======

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
    	Follow path error code. See ``FollowPath`` action for the enumerated set of error code definitions.

Example
-------

.. code-block:: xml

    <FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="precise_goal_checker" server_name="FollowPath" server_timeout="10" error_code_id="{follow_path_error_code}"/>
