.. _configuring_waypoint_follower:

Waypoint Follower
#################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower

The Waypoint Follower module implements a way of doing waypoint following using the NavigateToPose action server.
It will take in a set of ordered waypoints to follow and then try to navigate to them in order.
It also hosts a waypoint task executor plugin which can be used to perform custom behavior at a waypoint like waiting for user instruction, taking a picture, or picking up a box.
If a waypoint is not achievable, the ``stop_on_failure`` parameter will determine whether to continue to the next point or stop.

Parameters
**********

:stop_on_failure:

  ==== =======
  Type Default                                                   
  ---- -------
  bool true            
  ==== =======

  Description
    Whether to fail action task if a single waypoint fails. If false, will continue to next waypoint.

:loop_rate:

  ==== =======
  Type Default                                                   
  ---- -------
  int  20            
  ==== =======

  Description
    Rate to check for results from current navigation task.

:global_frame_id:

  ============== ========================
  Type           Default
  -------------- ------------------------
  string         'map'
  ============== ========================

  Description
    The name of the global coordinate frame published by robot_localization. Only used by the gps_waypoint_follower to
    convert GPS waypoints to this frame.

:action_server_result_timeout:

  ====== ======= ======= 
  Type   Default Unit
  ------ ------- -------
  double 900.0   seconds
  ====== ======= =======

  Description
    The timeout value (in seconds) for action servers to discard a goal handle if a result has not been produced. This used to default to
    15 minutes in rcl but was changed to 10 seconds in this `PR #1012 <https://github.com/ros2/rcl/pull/1012>`_, which may be less than
    some actions in Nav2 take to run. For most applications, this should not need to be adjusted as long as the actions within the server do not exceed this deadline. 
    This issue has been raised with OSRF to find another solution to avoid active goal timeouts for bookkeeping, so this is a semi-temporary workaround

:bond_heartbeat_period:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.1
  ============== =============================

  Description
    The lifecycle node bond mechanism publishing period (on the /bond topic). Disabled if inferior or equal to 0.0.

:waypoint_task_executor_plugin:

  ============== ========================
  Type           Default
  -------------- ------------------------
  string         'wait_at_waypoint'
  ============== ========================

  Description
    A plugin to define tasks to be executed when robot arrives to a waypoint.

  Note
    The plugin namespace defined needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        waypoint_follower:
          ros__parameters:
            waypoint_task_executor_plugin: "wait_at_waypoint"
            wait_at_waypoint:
              plugin: "nav2_waypoint_follower::WaitAtWaypoint"
              enabled: True
              waypoint_pause_duration: 0
    ..

Provided Plugins
****************
 The plugins listed below are inside the ``nav2_waypoint_follower`` namespace.

.. toctree::
  :maxdepth: 1

  nav2_waypoint_follower-plugins/wait_at_waypoint.rst
  nav2_waypoint_follower-plugins/photo_at_waypoint.rst
  nav2_waypoint_follower-plugins/input_at_waypoint.rst


Default Plugin
***************

  ========================== ===================================================
  Namespace                  Plugin
  -------------------------- ---------------------------------------------------
  "wait_at_waypoint"         "nav2_waypoint_follower::WaitAtWaypoint"
  ========================== ===================================================

Example
*******
.. code-block:: yaml

    waypoint_follower:
      ros__parameters:
        loop_rate: 20
        stop_on_failure: false
        waypoint_task_executor_plugin: "wait_at_waypoint"
          wait_at_waypoint:
            plugin: "nav2_waypoint_follower::WaitAtWaypoint"
            enabled: True
            waypoint_pause_duration: 0
