.. _configuring_map_server:

Map Server / Saver
##################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_map_server

The Map Server implements the server for handling the map load requests for the stack and host a map topic.
It also implements a map saver server which will run in the background and save maps based on service requests. There exists a map saver CLI similar to ROS 1 as well for a single map save.

Map Saver Parameters
********************

:save_map_timeout:

  ============== =======
  Type           Default
  -------------- -------
  int            2.0
  ============== =======

  Description
    Timeout to attempt saving the map (seconds).

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
    Occupied space minimum probability threshhold value for occupancy grid.

Map Server Parameters
*********************

:yaml_filename:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         N/A
  ============== =============================

  Description
    Path to map yaml file. Note from Rolling + Iron-Turtle forward: This parameter can set either from the yaml file or using the launch configuration parameter ``map``. If we set it on launch commandline / launch configuration default, we override the yaml default. If you would like the specify your map file in yaml, remove the launch default so it is not overridden in Nav2's default launch files. Before Iron: ``yaml_filename`` must be set in the yaml (even if a bogus value) so that our launch scripts can overwrite it with launch values.

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

Costmap Filter Info Server Parameters
*************************************

:type:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  int            0
  ============== =============================

  Description
    Type of costmap filter used. This is an enum for the type of filter this should be interpreted as. We provide the following pre-defined types:

    - 0: keepout zones / preferred lanes filter
    - 1: speed filter, speed limit is specified in % of maximum speed
    - 2: speed filter, speed limit is specified in absolute value (m/s)
    - 3: binary filter

:filter_info_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "costmap_filter_info"
  ============== =============================

  Description
    Topic to publish costmap filter information to.

:mask_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "filter_mask"
  ============== =============================

  Description
    Topic to publish filter mask to. The value of this parameter should be in accordance with ``topic_name`` parameter of Map Server tuned to filter mask publishing.

:base:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.0
  ============== =============================

  Description
    Base of ``OccupancyGrid`` mask value -> filter space value linear conversion which is being proceeded as:
    ``filter_space_value = base + multiplier * mask_value``

:multiplier:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         1.0
  ============== =============================

  Description
    Multiplier of ``OccupancyGrid`` mask value -> filter space value linear conversion which is being proceeded as:
    ``filter_space_value = base + multiplier * mask_value``

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
        save_map_timeout: 5.0
        free_thresh_default: 0.25
        occupied_thresh_default: 0.65

    costmap_filter_info_server:
      ros__parameters:
        type: 1
        filter_info_topic: "costmap_filter_info"
        mask_topic: "filter_mask"
        base: 0.0
        multiplier: 0.25
