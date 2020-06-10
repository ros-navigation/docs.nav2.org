.. _configuring_map_server:

Map Server / Saver
##################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_map_server

The Map Server implements the server for handling the map load requests for the stack and host a map topic.
It also implements a map saver server which will run in the background and save maps based on service requests. There exists a map saver CLI similar to ROS1 as well for a single map save.

Map Saver Parameters
********************

:save_map_timeout:

  ============== =======
  Type           Default
  -------------- -------
  int            2000   
  ============== =======

  Description
    Timeout to attempt saving the map (ms).

:free_thresh_default:

  ============== ==============
  Type           Default                                               
  -------------- --------------
  double         0.25            
  ============== ==============

  Description
    Free space maximum probability threshold value for occupancy grid.

:occupied_thresh_default:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.65 
  ============== =============================

  Description
    Occupied space minimum threshhold for occupancy grid.

Map Server Parameters
*********************

:yaml_filename:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         N/A            
  ============== =============================

  Description
    Path to map yaml file.

:topic_name:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "map"            
  ============== =============================

  Description
    Topic to publish loaded map to.

:frame_id:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "map"            
  ============== =============================

  Description
    Frame to publish loaded map in.

Example
*******
.. code-block:: yaml

    map_server:
      ros__parameters:
        yaml_filename: "turtlebot3_world.yaml"
        topic_name: "map"
        frame_id: "map"

    map_saver:
      ros__parameters:
        save_map_timeout: 5000
        free_thresh_default: 0.25
        occupied_thresh_default: 0.65
