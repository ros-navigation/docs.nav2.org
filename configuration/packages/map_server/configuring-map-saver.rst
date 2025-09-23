.. _configuring_map_saver:

Map Saver
#########

The map saver server runs in the background and saves maps based on service requests. There exists a map saver CLI similar to ROS 1 as well for a single map save.

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
    Occupied space minimum probability threshold value for occupancy grid.

:introspection_mode:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "disabled"
  ============== =============================

  Description
    The introspection mode for services and actions. Options are "disabled", "metadata", "contents".
