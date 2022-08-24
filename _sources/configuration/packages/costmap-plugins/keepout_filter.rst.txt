.. _keepout_filter:

Keepout Filter Parameters
=========================

Keepout Filter - is a Costmap Filter that enforces robot to avoid keepout areas or stay on preferred lanes, by updating corresponding costmap layer using filter mask information.

Note: As Costmap Filters does not have the inflation layer applied to them (since inflation is not sensible for a speed or other non-occupation zone type), it may be beneficial to add a separate inflation layer into the vector of filters when using only a keepout zone. Some planners (e.g. Smac Feasible) will use the cost of the center point for a collision checking optimization before doing full SE2 footprint checks. Without inflation, the planner will not respect the Keepout Zone on it extremities -- but will still respect it for the robot centers. If you wish to have a Keepout Zone for any part of the robot base while using a feasible planner, please enable the inflation layer.  

`<filter name>`: is the corresponding plugin name selected for this type.

:``<filter name>``.enabled:

  ====== =======
  Type   Default
  ------ -------
  bool   True
  ====== =======

  Description
    Whether it is enabled.

:``<filter name>``.filter_info_topic:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    Name of the incoming `CostmapFilterInfo <https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/msg/CostmapFilterInfo.msg>`_ topic having filter-related information. Published by Costmap Filter Info Server along with filter mask topic. For more details about Map and Costmap Filter Info servers configuration please refer to the :ref:`configuring_map_server` configuration page.

:``<filter name>``.transform_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.1
  ====== =======

  Description
    Time with which to post-date the transform that is published, to indicate that this transform is valid into the future. Used when filter mask and current costmap layer are in different frames.

Example
*******
.. code-block:: yaml

    global_costmap:
      global_costmap:
        ros__parameters:
          ...
          plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
          filters: ["keepout_filter"]
          ...
          keepout_filter:
            plugin: "nav2_costmap_2d::KeepoutFilter"
            enabled: True
            filter_info_topic: "/costmap_filter_info"
            transform_tolerance: 0.1
    ...
    local_costmap:
      local_costmap:
        ros__parameters:
          ...
          plugins: ["voxel_layer", "inflation_layer"]
          filters: ["keepout_filter"]
          ...
          keepout_filter:
            plugin: "nav2_costmap_2d::KeepoutFilter"
            enabled: True
            filter_info_topic: "/costmap_filter_info"
            transform_tolerance: 0.1
