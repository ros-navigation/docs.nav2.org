.. obstacle:

Obstacle Layer Parameters
=========================

This costmap layer implements a plugin that uses 2D raycasting for 2D lidars, depth, or other sensors. It contains a 2D costmap model within it that manages the planning space by the parameters specified below.

``<obstacle layer>`` is the corresponding plugin name selected for this type.

``<data source>`` is the corresponding observation source name for that sources parameters.

:``<obstacle layer>``.enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Whether it is enabled.

:``<obstacle layer>``.footprint_clearing_enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Clear any occupied cells under robot footprint.

:``<obstacle layer>``.max_obstacle_height:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 2.0            
  ====== =======

  Description
    Maximum height to add return to occupancy grid.

:``<obstacle layer>``.combination_method:

  ====== =======
  Type   Default                                                   
  ------ -------
  int    1            
  ====== =======

  Description
    Enum for method to add data to master costmap. Must be 0, 1 or 2, default to 1 (see below).

  | 0 - Overwrite: Overwrite master costmap with every valid observation.

  | 1 - Max: Sets the new value to the maximum of the master_grid's value and this layer's value. 
  | This is the default.
  
  | 2 - MaxWithoutUnknownOverwrite: Sets the new value to the maximum of the master_grid's
  | value and this layer's value. If the master value is NO_INFORMATION, it is NOT overwritten.
  | It can be used to make sure that the static map is the dominant source of information, and
  | prevent the robot to go through places that are not present in the static map.


:``<obstacle layer>``.observation_sources:

  ============== =======
  Type           Default                                                   
  -------------- -------
  vector<string> {""}            
  ============== =======

  Description
    namespace of sources of data.

:``<obstacle layer>``. ``<data source>``.topic:

  ====== =======
  Type   Default                                                   
  ------ -------
  string ""            
  ====== =======

  Description
    Topic of data.

  Relative topics will be relative to the node's parent namespace.
  For example, if you specify `topic: scan` in the `obstacle_layer` of a `local_costmap` and you launch your bringup with a `tb4` namespace:

  * User chosen namespace is `tb4`.
  * User chosen topic is `scan`.
  * Topic will be remapped to `/tb4/scan` without `local_costmap`.
  * Use global topic `/scan` if you do not wish the node namespace to apply.

:``<obstacle layer>``. ``<data source>``.sensor_frame:

  ====== =======
  Type   Default                                                   
  ------ -------
  string ""            
  ====== =======

  Description
    Frame of sensor, to use if not provided by message. If empty, uses message frame_id.

:``<obstacle layer>``. ``<data source>``.observation_persistence:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    How long to store messages in a buffer to add to costmap before removing them (s).

:``<obstacle layer>``. ``<data source>``.expected_update_rate:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Expected rate to get new data from sensor.

:``<obstacle layer>``. ``<data source>``.data_type:

  ====== ===========
  Type   Default                                                   
  ------ -----------
  string "LaserScan"            
  ====== ===========

  Description
    Data type of input, LaserScan or PointCloud2.

:``<obstacle layer>``. ``<data source>``.min_obstacle_height:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Minimum height to add return to occupancy grid.

:``<obstacle layer>``. ``<data source>``.max_obstacle_height:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Maximum height to add return to occupancy grid.

:``<obstacle layer>``. ``<data source>``.inf_is_valid:

  ====== =======
  Type   Default                                                   
  ------ -------
  bool   False            
  ====== =======

  Description
    Are infinite returns from laser scanners valid measurements to raycast.

:``<obstacle layer>``. ``<data source>``.marking:

  ====== =======
  Type   Default                                                   
  ------ -------
  bool   True            
  ====== =======

  Description
    Whether source should mark in costmap.

:``<obstacle layer>``. ``<data source>``.clearing:

  ====== =======
  Type   Default                                                   
  ------ -------
  bool   False            
  ====== =======

  Description
    Whether source should raytrace clear in costmap.

:``<obstacle layer>``. ``<data source>``.obstacle_max_range:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 2.5            
  ====== =======

  Description
    Maximum range to mark obstacles in costmap.

:``<obstacle layer>``. ``<data source>``.obstacle_min_range:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0           
  ====== =======

  Description
    Minimum range to mark obstacles in costmap.

:``<obstacle layer>``. ``<data source>``.raytrace_max_range:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 3.0            
  ====== =======

  Description
    Maximum range to raytrace clear obstacles from costmap.

:``<obstacle layer>``. ``<data source>``.raytrace_min_range:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Minimum range to raytrace clear obstacles from costmap.
