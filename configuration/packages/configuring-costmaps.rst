.. _configuring_cosmaps:

Costmap 2D
##########

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_costmap_2d

The Costmap 2D package implements a 2D grid-based costmap for environmental representations and a number of sensor processing plugins.
It is used in the planner and controller servers for creating the space to check for collisions or higher cost areas to negotiate around. 

Costmap2D ROS Parameters
************************

:always_send_full_costmap:

  ============== =======
  Type           Default
  -------------- -------
  bool           False   
  ============== =======

  Description
    Whether to send full costmap every update, rather than updates.

:footprint_padding:

  ============== =======
  Type           Default
  -------------- -------
  double         0.01   
  ============== =======

  Description
    Amount to pad footprint (m).

:footprint:

  ============== =======
  Type           Default
  -------------- -------
  vector<double> {}   
  ============== =======

  Description
    Ordered set of footprint points, must be closed set.

:global_frame:

  ============== =======
  Type           Default
  -------------- -------
  string         "map"   
  ============== =======

  Description
    Reference frame.

:height:

  ============== =======
  Type           Default
  -------------- -------
  int            5   
  ============== =======

  Description
    Height of costmap (m).

:width:

  ============== =======
  Type           Default
  -------------- -------
  int            5   
  ============== =======

  Description
    Width of costmap (m).

:lethal_cost_threshold:

  ============== =======
  Type           Default
  -------------- -------
  int            100    
  ============== =======

  Description
    Minimum cost of an occupancy grid map to be considered a lethal obstacle.

:map_topic:

  ============== =======
  Type           Default
  -------------- -------
  string         "map"   
  ============== =======

  Description
    Topic of map from map_server or SLAM.

:observation_sources:

  ============== =======
  Type           Default
  -------------- -------
  vector<string> {""}   
  ============== =======

  Description
    List of sources of sensors, to be used if not specified in plugin specific configurations.

:origin_x:

  ============== =======
  Type           Default
  -------------- -------
  double         0.0   
  ============== =======

  Description
    X origin of the costmap relative to width (m).

:origin_y:

  ============== =======
  Type           Default
  -------------- -------
  double         0.0   
  ============== =======

  Description
    Y origin of the costmap relative to height (m).

:plugin_names:

  ============== =====================================================
  Type           Default                                              
  -------------- -----------------------------------------------------
  vector<string> {"static_layer", "obstacle_layer", "inflation_layer"}   
  ============== =====================================================

  Description
    List of mapped plugin names for parameter namespaces and names.

:plugin_types:

  ============== =====================================================================================================
  Type           Default                                                                                              
  -------------- -----------------------------------------------------------------------------------------------------
  vector<string> {"nav2_costmap_2d::StaticLayer", "nav2_costmap_2d::ObstacleLayer", "nav2_costmap_2d::InflationLayer"}   
  ============== =====================================================================================================

  Description
    List of registered plugins to map to names and load.

:publish_frequency:

  ============== =======
  Type           Default
  -------------- -------
  double         1.0   
  ============== =======

  Description
    Frequency to publish costmap to topic.

:resolution:

  ============== =======
  Type           Default
  -------------- -------
  double         0.1   
  ============== =======

  Description
    Resolution of 1 pixel of the costmap, in meters.

:robot_base_frame:

  ============== ===========
  Type           Default    
  -------------- -----------
  string         "base_link"   
  ============== ===========

  Description
    Robot base frame.

:robot_radius:

  ============== =======
  Type           Default
  -------------- -------
  double         0.1   
  ============== =======

  Description
    Robot radius to use, if footprint coordinates not provided.

:rolling_window:

  ============== =======
  Type           Default
  -------------- -------
  bool           False   
  ============== =======

  Description
    Whether costmap should roll with robot base frame.

:track_unknown_space:

  ============== =======
  Type           Default
  -------------- -------
  bool           False   
  ============== =======

  Description
    If false, treats unknown space as free space, else as unknown space.

:transform_tolerance:

  ============== =======
  Type           Default
  -------------- -------
  double         0.3   
  ============== =======

  Description
    TF transform tolerance.

:trinary_costmap:

  ============== =======
  Type           Default
  -------------- -------
  bool           True   
  ============== =======

  Description
    If occupancy grid map should be interpreted as only 3 values (free, occupied, unknown) or with its stored values.

:unknown_cost_value:

  ============== =======
  Type           Default
  -------------- -------
  int            255    
  ============== =======

  Description
    Cost of unknown space if tracking it.

:update_frequency:

  ============== =======
  Type           Default
  -------------- -------
  double         5.0   
  ============== =======

  Description
    Costmap update frequency.

:use_maximum:

  ============== =======
  Type           Default
  -------------- -------
  bool           False   
  ============== =======

  Description
    whether when combining costmaps to use the maximum cost or override.

:clearable_layers:

  ============== =====================================================
  Type           Default                                              
  -------------- -----------------------------------------------------
  vector<string> {"obstacle_layer"}   
  ============== =====================================================

  Description
    Layers that may be cleared using the clearing service.


Static Layer Parameters
***********************

``<static layer>`` is the corresponding plugin name selected for this type.

:``<static layer>``.enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Whether it is enabled.

:``<static layer>``.subscribe_to_updates:

  ==== =======
  Type Default                                                   
  ---- -------
  bool False            
  ==== =======

  Description
    Subscribe to static map updates after receiving first.

:``<static layer>``.map_subscribe_transient_local:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    QoS settings for map topic.

:``<static layer>``.transform_tolerance:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    TF tolerance.

inflation Layer Parameters
**************************

``<inflation layer>`` is the corresponding plugin name selected for this type.


:``<inflation layer>``.enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Whether it is enabled.

:``<inflation layer>``.inflation_radius:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.55            
  ====== =======

  Description
    Radius to inflate costmap around lethal obstacles.

:``<inflation layer>``.cost_scaling_factor:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 10.0            
  ====== =======

  Description
    Exponential decay factor across inflation radius.


:``<inflation layer>``.inflate_unknown:

  ==== =======
  Type Default                                                   
  ---- -------
  bool False            
  ==== =======

  Description
    Whether to inflate unknown cells as if lethal.


:``<inflation layer>``.inflate_around_unknown:

  ==== =======
  Type Default                                                   
  ---- -------
  bool False            
  ==== =======

  Description
    Whether to inflate unknown cells.

Obstacle Layer Parameters
*************************

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
    Enum for method to add data to master costmap, default to maximum.

:``<obstacle layer>``.observation_sources:

  ============== =======
  Type           Default                                                   
  -------------- -------
  vector<string> {""}            
  ============== =======

  Description
    namespace of sources of data.

:``<obstacle layer>``.``<data source>``.topic:

  ====== =======
  Type   Default                                                   
  ------ -------
  string ""            
  ====== =======

  Description
    Topic of data.

:``<obstacle layer>``.``<data source>``.sensor_frame:

  ====== =======
  Type   Default                                                   
  ------ -------
  string ""            
  ====== =======

  Description
    Frame of sensor, to use if not provided by message. If empty, uses message frame_id.

:``<obstacle layer>``.``<data source>``.observation_persistence:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    How long to store messages in a buffer to add to costmap before removing them (s).

:``<obstacle layer>``.``<data source>``.expected_update_rate:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Expected rate to get new data from sensor.

:``<obstacle layer>``.``<data source>``.data_type:

  ====== ===========
  Type   Default                                                   
  ------ -----------
  string "LaserScan"            
  ====== ===========

  Description
    Data type of input, LaserScan or PointCloud2.

:``<obstacle layer>``.``<data source>``.min_obstacle_height:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Minimum height to add return to occupancy grid.

:``<obstacle layer>``.``<data source>``.max_obstacle_height:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Maximum height to add return to occupancy grid.

:``<obstacle layer>``.``<data source>``.inf_is_valid:

  ====== =======
  Type   Default                                                   
  ------ -------
  bool   False            
  ====== =======

  Description
    Are infinite returns from laser scanners valid measurements to raycast.

:``<obstacle layer>``.``<data source>``.marking:

  ====== =======
  Type   Default                                                   
  ------ -------
  bool   True            
  ====== =======

  Description
    Whether source should mark in costmap.

:``<obstacle layer>``.``<data source>``.clearing:

  ====== =======
  Type   Default                                                   
  ------ -------
  bool   False            
  ====== =======

  Description
    Whether source should raytrace clear in costmap.

:``<obstacle layer>``.``<data source>``.obstacle_range:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 2.5            
  ====== =======

  Description
    Maximum range to mark obstacles in costmap.

:``<obstacle layer>``.``<data source>``.raytrace_range:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 3.0            
  ====== =======

  Description
    Maximum range to raytrace clear obstacles from costmap.

Voxel Layer Parameters
**********************

``<voxel layer>`` is the corresponding plugin name selected for this type.

``<data source>`` is the corresponding observation source name for that sources parameters.

:``<voxel layer>``.enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Whether it is enabled.

:``<voxel layer>``.footprint_clearing_enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Clear any occupied cells under robot footprint.

:``<voxel layer>``.max_obstacle_height:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 2.0            
  ====== =======

  Description
    Maximum height to add return to occupancy grid.

:``<voxel layer>``.z_voxels:

  ====== =======
  Type   Default                                                   
  ------ -------
  int    10            
  ====== =======

  Description
    Number of voxels high to mark, maximum 16.

:``<voxel layer>``.origin_z:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Where to start marking voxels (m).

:``<voxel layer>``.z_resolution:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.2            
  ====== =======

  Description
    Resolution of voxels in height (m).

:``<voxel layer>``.unknown_threshold:

  ====== =======
  Type   Default                                                   
  ------ -------
  int    15            
  ====== =======

  Description
    Minimum number of empty voxels in a column to mark as unknown in 2D occupancy grid.

:``<voxel layer>``.mark_threshold:

  ====== =======
  Type   Default                                                   
  ------ -------
  int    0            
  ====== =======

  Description
    Minimum number of voxels in a column to mark as occupied in 2D occupancy grid.

:``<voxel layer>``.combination_method:

  ====== =======
  Type   Default                                                   
  ------ -------
  int    1            
  ====== =======

  Description
    Enum for method to add data to master costmap, default to maximum.

:``<voxel layer>``.publish_voxel_map:

  ==== =======
  Type Default                                                   
  ---- -------
  bool False            
  ==== =======

  Description
    Whether to publish 3D voxel grid for debug, computationally expensive.

:``<voxel layer>``.observation_sources:

  ============== =======
  Type           Default                                                   
  -------------- -------
  vector<string> {""}            
  ============== =======

  Description
    namespace of sources of data.

:``<voxel layer>``.``<data source>``.topic:

  ====== =======
  Type   Default                                                   
  ------ -------
  string ""            
  ====== =======

  Description
    Topic of data.

:``<voxel layer>``.``<data source>``.sensor_frame:

  ====== =======
  Type   Default                                                   
  ------ -------
  string ""            
  ====== =======

  Description
    Frame of sensor, to use if not provided by message. If empty, uses message frame_id.

:``<voxel layer>``.``<data source>``.observation_persistence:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    How long to store messages in a buffer to add to costmap before removing them (s).

:``<voxel layer>``.``<data source>``.expected_update_rate:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Expected rate to get new data from sensor.

:``<voxel layer>``.``<data source>``.data_type:

  ====== ===========
  Type   Default                                                   
  ------ -----------
  string "LaserScan"            
  ====== ===========

  Description
    Data type of input, LaserScan or PointCloud2.

:``<voxel layer>``.``<data source>``.min_obstacle_height:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Minimum height to add return to occupancy grid.

:``<voxel layer>``.``<data source>``.max_obstacle_height:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    Maximum height to add return to occupancy grid.

:``<voxel layer>``.``<data source>``.inf_is_valid:

  ====== =======
  Type   Default                                                   
  ------ -------
  bool   False            
  ====== =======

  Description
    Are infinite returns from laser scanners valid measurements to raycast.

:``<voxel layer>``.``<data source>``.marking:

  ====== =======
  Type   Default                                                   
  ------ -------
  bool   True            
  ====== =======

  Description
    Whether source should mark in costmap.

:``<voxel layer>``.``<data source>``.clearing:

  ====== =======
  Type   Default                                                   
  ------ -------
  bool   False            
  ====== =======

  Description
    Whether source should raytrace clear in costmap.

:``<voxel layer>``.``<data source>``.obstacle_range:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 2.5            
  ====== =======

  Description
    Maximum range to mark obstacles in costmap.

:``<voxel layer>``.``<data source>``.raytrace_range:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 3.0            
  ====== =======

  Description
    Maximum range to raytrace clear obstacles from costmap.


Example
*******
.. code-block:: yaml

    global_costmap:
      global_costmap:
        ros__parameters:
          footprint_padding: 0.03
          update_frequency: 1.0
          publish_frequency: 1.0
          global_frame: map
          robot_base_frame: base_link
          use_sim_time: True
          plugin_names: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
          plugin_types: ["nav2_costmap_2d::StaticLayer", "nav2_costmap_2d::ObstacleLayer", "nav2_costmap_2d::VoxelLayer", "nav2_costmap_2d::InflationLayer"]
          robot_radius: 0.22 # radius set and used, so no footprint points
          resolution: 0.05
          obstacle_layer:
            enabled: True
            observation_sources: scan
            footprint_clearing_enabled: true
            max_obstacle_height: 2.0
            combination_method: 1
            scan:
              topic: /scan
              obstacle_range: 2.5
              raytrace_range: 3.0
              max_obstacle_height: 2.0
              min_obstacle_height: 0.0
              clearing: True
              marking: True
              data_type: "LaserScan"
              inf_is_valid: false
          voxel_layer:
            enabled: True
            footprint_clearing_enabled: true
            max_obstacle_height: 2.0
            publish_voxel_map: True
            origin_z: 0.0
            z_resolution: 0.05
            z_voxels: 16
            max_obstacle_height: 2.0
            unknown_threshold: 15
            mark_threshold: 0
            observation_sources: pointcloud
            combination_method: 1
            pointcloud:  # no frame set, uses frame from message
              topic: /intel_realsense_r200_depth/points
              max_obstacle_height: 2.0
              min_obstacle_height: 0.0
              obstacle_range: 2.5
              raytrace_range: 3.0
              clearing: True
              marking: True
              data_type: "PointCloud2"
          static_layer:
            map_subscribe_transient_local: True
            enabled: true
            subscribe_to_updates: true
            transform_tolerance: 0.1
          inflation_layer:
            enabled: true
            inflation_radius: 0.55
            cost_scaling_factor: 1.0
            inflate_unknown: false
            inflate_around_unknown: true
          always_send_full_costmap: True


    local_costmap:
      local_costmap:
        ros__parameters:
          update_frequency: 5.0
          publish_frequency: 2.0
          global_frame: odom
          robot_base_frame: base_link
          use_sim_time: True
          rolling_window: true
          width: 3
          height: 3
          resolution: 0.05
