.. _configuring_cosmaps:

Costmap 2D
##########

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d

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
  vector<double> "[]"   
  ============== =======

  Description
    Ordered set of footprint points passed in as a string, must be closed set. For example, the following defines a square base with side lengths of 0.2 meters `footprint: "[ [0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1] ]"`.

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
  string         ""   
  ============== =======

  Description
    List of sources of sensors as a string, to be used if not specified in plugin specific configurations. Ex. "static_layer stvl_layer"

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

:plugins:

  ============== =====================================================
  Type           Default
  -------------- -----------------------------------------------------
  vector<string> {"static_layer", "obstacle_layer", "inflation_layer"}
  ============== =====================================================

  Description
    List of mapped plugin names for parameter namespaces and names.

  Note
    Each plugin namespace defined in this list needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        local_costmap:
          ros__parameters:
            plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
            obstacle_layer:
              plugin: "nav2_costmap_2d::ObstacleLayer"
            voxel_layer:
              plugin: "nav2_costmap_2d::VoxelLayer"
            inflation_layer:
              plugin: "nav2_costmap_2d::InflationLayer"
    ..

:filters:

  ============== =======
  Type           Default
  -------------- -------
  vector<string> {}
  ============== =======

  Description
    List of mapped costmap filter names for parameter namespaces and names.

  Note
    Costmap filters are also loadable plugins just as ordinary costmap layers. This separation is made to avoid plugin and filter interference and places these filters on top of the combined layered costmap. As  with plugins, each costmap filter namespace defined in this list needs to have a :code:`plugin` parameter defining the type of filter plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        local_costmap:
          ros__parameters:
            filters: ["keepout_filter", "speed_filter"]
            keepout_filter:
              plugin: "nav2_costmap_2d::KeepoutFilter"
            speed_filter:
              plugin: "nav2_costmap_2d::SpeedFilter"
    ..

Default Plugins
***************

When the :code:`plugins` parameter is not overridden, the following default plugins are loaded:

  ================= =====================================================
  Namespace         Plugin
  ----------------- -----------------------------------------------------
  "static_layer"    "nav2_costmap_2d::StaticLayer"
  ----------------- -----------------------------------------------------
  "obstacle_layer"  "nav2_costmap_2d::ObstacleLayer"
  ----------------- -----------------------------------------------------
  "inflation_layer" "nav2_costmap_2d::InflationLayer"
  ================= =====================================================

Plugin Parameters
*****************

.. toctree::
  :maxdepth: 1

  costmap-plugins/static.rst
  costmap-plugins/inflation.rst
  costmap-plugins/obstacle.rst
  costmap-plugins/voxel.rst
  costmap-plugins/range.rst
  costmap-plugins/denoise.rst

Costmap Filters Parameters
**************************

.. toctree::
  :maxdepth: 1

  costmap-plugins/keepout_filter.rst
  costmap-plugins/speed_filter.rst
  costmap-plugins/binary_filter.rst

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
          robot_radius: 0.22 # radius set and used, so no footprint points
          resolution: 0.05
          plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
          obstacle_layer:
            plugin: "nav2_costmap_2d::ObstacleLayer"
            enabled: True
            observation_sources: scan
            footprint_clearing_enabled: true
            max_obstacle_height: 2.0
            combination_method: 1
            scan:
              topic: /scan
              obstacle_max_range: 2.5
              obstacle_min_range: 0.0
              raytrace_max_range: 3.0
              raytrace_min_range: 0.0
              max_obstacle_height: 2.0
              min_obstacle_height: 0.0
              clearing: True
              marking: True
              data_type: "LaserScan"
              inf_is_valid: false
          voxel_layer:
            plugin: "nav2_costmap_2d::VoxelLayer"
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
              obstacle_max_range: 2.5
              obstacle_min_range: 0.0
              raytrace_max_range: 3.0
              raytrace_min_range: 0.0
              clearing: True
              marking: True
              data_type: "PointCloud2"
          static_layer:
            plugin: "nav2_costmap_2d::StaticLayer"
            map_subscribe_transient_local: True
            enabled: true
            subscribe_to_updates: true
            transform_tolerance: 0.1
          inflation_layer:
            plugin: "nav2_costmap_2d::InflationLayer"
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
