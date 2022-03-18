.. _configuring_nav2_waypoint_follower_wait_at_waypoint_plugin:

WaitAtWaypoint
==============

Lets robot to pause for a specified amount of time after reaching each waypoints.

Parameters
**********

``<nav2_waypoint_follower plugin>``: nav2_waypoint_follower plugin name defined in the **waypoint_task_executor_plugin_id** parameter in :ref:`configuring_waypoint_follower`.

:``<nav2_waypoint_follower plugin>``.enabled:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           true           
  ============== =============================

  Description
    Whether waypoint_task_executor plugin is enabled.


:``<nav2_waypoint_follower plugin>``.waypoint_pause_duration:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  int            0           
  ============== =============================

  Description
    Amount of time in milliseconds, for robot to sleep/wait after each waypoint is reached. If zero, robot will directly continue to next waypoint.

