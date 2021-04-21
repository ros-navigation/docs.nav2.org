.. _speed_filter:

Speed Filter Parameters
=======================

Speed Filter - is a Costmap Filter that restricting maximum velocity of robot. The areas where robot should slow down and values of maximum allowed velocities are encoded at filter mask. Filter mask published by Map Server, goes in a pair with filter info topic published by Costmap Filter Info Server. Speed Filter itself publishes a speed restricting messages which are targeted for a Controller in order to make the robot to not exceed the required velocity.

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="800" height="450" src="https://www.youtube.com/embed/pq0r0lqi0Sc?autoplay=1" frameborder="1" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
      </div>
    </h1>

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

:``<filter name>``.speed_limit_topic:

  ====== =============
  Type   Default
  ------ -------------
  string "speed_limit"
  ====== =============

  Description
    Topic to publish speed limit to. The `messages <https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/msg/SpeedLimit.msg>`_ have the following fields' meaning:

    - ``percentage``: speed limit is expressed in percentage if ``true`` or in absolute values in ``false`` case. This parameter is set depending on ``type`` field of ``CostmapFilterInfo`` message.

    - ``speed_limit``: non-zero values show maximum allowed speed expressed in a percent of maximum robot speed or in absolute value depending on ``percentage`` value. Zero value means no speed restriction (independently on ``percentage``). ``speed_limit`` is being linearly converted from ``OccupancyGrid`` filter mask value as: ``speed_limit = base + multiplier * mask_value``, where ``base`` and ``multiplier`` coefficients are taken from ``CostmapFilterInfo`` message.

      Note
        ``speed_limit`` expressed in a percent should belong to ``(0.0 .. 100.0]`` range.

    This topic will be used by a Controller Server. Please refer to :ref:`configuring_controller_server` configuration page to set it appropriately.


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
        filters: ["speed_filter"]
        ...
        speed_filter:
          plugin: "nav2_costmap_2d::SpeedFilter"
          enabled: True
          filter_info_topic: "/costmap_filter_info"
          speed_limit_topic: "/speed_limit"
          transform_tolerance: 0.1
