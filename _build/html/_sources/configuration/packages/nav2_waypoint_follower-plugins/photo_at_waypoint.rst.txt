.. _configuring_nav2_waypoint_follower_photo_at_waypoint_plugin:

PhotoAtWaypoint
===============

Run-time plugin that takes photos at waypoint arrivals when using waypoint follower node. Saves the taken photos to specified directory. The name for taken photos are determined by 
the waypoint index and timestamp(seconds). For instance ``/home/atas/0_1602582820.png`` is an sample taken image, where ``0_1602582820`` is the file name determined by waypoint 
index and time stamp. The leading digit in file name implies the waypoint index and the rest of digits at righthand side imples the time stamp when the photo was taken.

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
  string         "/tmp/waypoint_images"          
  ============== =============================

  Description
    Path to directory to save taken photos at waypoint arrivals.

:``<nav2_waypoint_follower plugin>``.image_format:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "png"          
  ============== =============================

  Description
    Desired image format.
