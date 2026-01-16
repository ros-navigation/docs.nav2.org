.. _bt_navigate_through_poses_action:

NavigateThroughPoses
====================

Invokes the NavigateThroughPoses ROS 2 action server, which is implemented by the bt_navigator_ module.

.. _bt_navigator: https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator

Input Ports
-----------

:goals:

  ==================== =======
  Type                 Default
  -------------------- -------
  nav_msgs::msg::Goals   N/A
  ==================== =======

  Description
      Goal poses. Takes in a blackboard variable, e.g. "{goals}".

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

:behavior_tree:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
        Behavior tree absolute path or ID. If none is specified, NavigateThroughPoses action server uses a default behavior tree.

Output Ports
------------

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A
  ============== =======

  Description
        The lowest error code in the list of the `error_code_name_prefixes` parameter.

:error_msg:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A
  ============== =======

  Description
        The error message associated with the lowest error code in the list of the `error_code_name_prefixes` parameter.

Example
-------

.. code-block:: xml

  <NavigateThroughPoses goals="{goals}" server_name="navigate_through_poses" server_timeout="10"
                        error_code_id="{navigate_through_poses_error_code}" error_msg="{navigate_through_poses_error_msg}"
                        behavior_tree="<some-path>/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"/>
