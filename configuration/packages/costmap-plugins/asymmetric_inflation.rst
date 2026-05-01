.. asymmetric_inflation:

Asymmetric Inflation Layer Parameters
======================================

This layer implements an asymmetric variant of the inflation layer, biasing the robot's path toward one side of the corridor by raising obstacle costs asymmetrically around the planned path.


``<asymmetric inflation layer>`` is the corresponding plugin name selected for this type.


:``<asymmetric inflation layer>``.enabled:

  ==== =======
  Type Default
  ---- -------
  bool True
  ==== =======

  Description
    Whether it is enabled.

:``<asymmetric inflation layer>``.inflation_radius:

  ====== =======
  Type   Default
  ------ -------
  double 2.0
  ====== =======

  Description
    Radius to inflate costmap around lethal obstacles.

:``<asymmetric inflation layer>``.cost_scaling_factor:

  ====== =======
  Type   Default
  ------ -------
  double 4.0
  ====== =======

  Description
    Exponential decay factor across inflation radius.

:``<asymmetric inflation layer>``.asymmetry_factor:

  ====== =======
  Type   Default
  ------ -------
  double 0.75
  ====== =======

  Description
    Signed bias in to influence asymmetry. Must be in the range (-1, 1).

    a value > 0 biases to the right of the path, < 0 to the left



:``<asymmetric inflation layer>``.inflate_around_unknown:

  ==== =======
  Type Default
  ---- -------
  bool False
  ==== =======

  Description
    Whether to treat unknown cells as lethal for inflation purposes.

:``<asymmetric inflation layer>``.plan_topic:

  ====== =======
  Type   Default
  ------ -------
  string "plan"
  ====== =======

  Description
    Topic on which to receive the global path (``nav_msgs/msg/Path``).

:``<asymmetric inflation layer>``.goal_distance_threshold:

  ====== =======
  Type   Default
  ------ -------
  double 1.5
  ====== =======

  Description
    Distance to the goal (m) below which asymmetric inflation is disabled.

    This prevents oscillations when the robot is approaching the target pose.

:``<asymmetric inflation layer>``.neutral_threshold:

  ====== =======
  Type   Default
  ------ -------
  double 2.0
  ====== =======

  Description
    Maximum perpendicular distance (m) from the path centreline.

    Obstacles farther than this distance are ignored.


Usage Note
----------

``AsymmetricInflationLayer`` reads the symmetric cost baseline written by
``nav2_costmap_2d::InflationLayer``.
It must appear **after** ``InflationLayer`` in the ``plugins`` list:

.. code-block:: yaml

    local_costmap:
      ros__parameters:
        plugins: ["obstacle_layer", "inflation_layer", "asymmetric_inflation_layer"]
        obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          observation_sources: scan
          scan:
            topic: scan
            max_obstacle_height: 2.0
            clearing: true
            marking: true
            data_type: "LaserScan"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
          obstacle_min_range: 0.0
        inflation_layer:
          plugin: "nav2_costmap_2d::InflationLayer"
          inflation_radius: 1.0
          cost_scaling_factor: 5.0
        asymmetric_inflation_layer:
          plugin: "nav2_costmap_2d::AsymmetricInflationLayer"
          inflation_radius: 3.0
          cost_scaling_factor: 4.0
          asymmetry_factor: 0.75
          plan_topic: "plan"
          goal_distance_threshold: 1.5
          neutral_threshold: 2.0
