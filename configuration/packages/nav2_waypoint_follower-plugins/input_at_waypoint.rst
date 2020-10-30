.. _configuring_nav2_waypoint_follower_input_at_waypoint_plugin:

InputAtWaypoint
===============

Lets robot to wait for external input, with timeout, at a waypoint.

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


:``<nav2_waypoint_follower plugin>``.timeout:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         10.0           
  ============== =============================

  Description
    Amount of time in seconds to wait for user input before moving on to the next waypoint.

:``<nav2_waypoint_follower plugin>``.input_topic:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "input_at_waypoint/input"           
  ============== =============================

  Description
    Topic input is published to to indicate to move to the next waypoint, in `std_msgs/Empty`.
