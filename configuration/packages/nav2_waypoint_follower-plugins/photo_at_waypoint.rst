.. _configuring_nav2_waypoint_follower_waypoint_task_execution_plugin:

PhotoAtWaypoint
=================

Run-time plugin that takes photos at waypoint arrivals when using waypoint follower node. Saves the taken photos to specified directory.

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


:``<nav2_waypoint_follower plugin>``.camera_image_topic_name:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "/camera/color/image_raw"        
  ============== =============================

  Description
     Camera image topic name to susbcribe

:``<nav2_waypoint_follower plugin>``.save_images_dir:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "/home/username/"          
  ============== =============================

  Description
    Path to directory to save taken photos at waypoint arrivals.

:``<nav2_waypoint_follower plugin>``.image_format:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         ".png"          
  ============== =============================

  Description
    Desired image format.
