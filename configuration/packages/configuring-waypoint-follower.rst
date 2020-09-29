.. _configuring_waypoint_follower:

Waypoint Follower
#################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower

The Waypoint Follower module implements a way of doing waypoint following using the NavigateToPose action server.
It will take in a set of ordered waypoints to follow and then try to navigate to them in order.
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

:waypoint_task_executor_plugin:

  ============== ========================
  Type           Default
  -------------- ------------------------
  string         'waypoint_task_executor'
  ============== ========================

  Description
    A plugin to define tasks to be executed when robot arrives to a waypoint.

  Note
    The plugin namespace defined needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        waypoint_follower:
          ros__parameters:
            waypoint_task_executor_plugin: "waypoint_task_executor"
            waypoint_task_executor:
              plugin: "nav2_waypoint_follower::WaitAtWaypoint"
    ..

:enabled:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           true           
  ============== =============================

  Description
    Whether waypoint_task_executor plugin is enabled.

:waypoint_pause_duration:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  int            0           
  ============== =============================

  Description
    Amount of time in milliseconds, for robot to sleep/wait after each waypoint is reached. If zero, robot will directly continue to next waypoint.


Provided Plugins
****************
 The plugins listed below are inside the ``nav2_waypoint_follower`` namespace.

.. toctree::
  :maxdepth: 1

  nav2_waypoint_follower-plugins/wait_at_waypoint.rst


Default Plugins
***************

  ========================== ===================================================
  Namespace                  Plugin
  -------------------------- ---------------------------------------------------
  "waypoint_task_executor"   "nav2_waypoint_follower::WaitAtWaypoint"
  ========================== ===================================================

Example
*******
.. code-block:: yaml

    waypoint_follower:
      ros__parameters:
        loop_rate: 20
        stop_on_failure: false
        waypoint_task_executor_plugin: "waypoint_task_executor"
          waypoint_task_executor:
            plugin: "nav2_waypoint_follower::WaitAtWaypoint"
              enabled: True
              waypoint_pause_duration: 0

