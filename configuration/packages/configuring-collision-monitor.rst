.. _configuring_collision_monitor:

Collision Monitor
#################

Collision Monitor - is an independent layer in Nav2 providing an additional level of robot safety assurance.
It allows performing robot collision avoidance by monitoring the obstacles in surrounding environments laying in collision proximity to the robot.

See the package's ``README`` for more complete information.

Features
********

The following models of safety behaviors are employed by Collision Monitor:

- **Stop model**: Define a safety area surrounding the robot and a point threshold. If more that ``max_points`` appear inside this area, stop the robot until the obstacles will disappear.
- **Slowdown model**: Define a safety area around the robot and slow the maximum speed for a ``slowdown_ratio``, if more than ``max_points`` points will appear inside the area.
- **Approach model**: With the current robot speed, estimate the time to collision to points obtained from sensors. If the time is less than ``time_before_collision`` seconds, slow the robot until it will be equal to that time. The effect here would be to keep the robot always ``time_before_collision`` seconds from a collision and continuously scale down its speeds.

The safety area around the robot can take the following shapes:

- Arbitrary user-defined polygon around the robot for usage in stop and slowdown models.
- Robot footprint polygon, which is used in the approach behavior model only.
- Circle, used in all models: for stop and slowdown models as a safety area, for approach model as a robot footprint. Circle is made for the best performance and could be used in the cases where the safety area or robot could be approximated by round shape.

NOTE: Although safety behavior models are not intended to be used simultaneously (e.g. stop model should not be crossed with approach one), it is not prohibited to. Collision Monitor allows setting simultaneously multiple shapes with different behavior models.

All shapes (``Polygon`` and ``Circle``) are derived from base ``Polygon`` class, so without loss of generality they would be called as "polygons". Subscribed footprint is also having the same properties as other polygons, but it is being obtained from ``nav2_costmap_2d::FootprintSubscriber``. 

The obstacle points are being obtained from different data sources. Collision Monitor is subscribed to:

- Laser scanners (``sensor_msgs::msg::LaserScan`` messages)
- PointClouds (``sensor_msgs::msg::PointCloud2`` messages)
- IR/Sonars (``sensor_msgs::msg::Range`` messages)

Parameters
**********

:base_frame_id:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "base_footprint"
  ============== =============================

  Description:
    Robot base frame.

:odom_frame_id:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "odom"
  ============== =============================

  Description:
    Which frame to use for odometry.

:cmd_vel_in_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "cmd_vel_raw"
  ============== =============================

  Description:
    Input ``cmd_vel`` topic with desired robot velocity.

:cmd_vel_out_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "cmd_vel"
  ============== =============================

  Description:
    Output ``cmd_vel`` topic with output produced by Collision Monitor velocities.

:transform_tolerance:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.1
  ============== =============================

  Description
    Time with which to post-date the transform that is published, to indicate that this transform is valid into the future.

:source_timeout:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         2.0
  ============== =============================

  Description:
    Maximum time interval in which source data is considered as valid.

:stop_pub_timeout:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         1.0
  ============== =============================

  Description:
    Timeout, after which zero-velocity ceases to be published. It could be used for other overrode systems outside Nav2 are trying to bring the robot out of a state close to a collision, or to allow a standing robot to go into sleep mode.

:polygons:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  vector<string> N/A
  ============== =============================

  Description:
    List of safety area shapes (stop/slowdown bounding boxes, footprint, approach circle, etc...). Causes an error, if not specialized.


:observation_sources:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  vector<string> N/A
  ============== =============================

  Description:
    List of data sources (llaser scanners, pointclouds, etc...). Causes an error, if not specialized.

Polygons parameters
===================

``<polygon name>`` is the corresponding polygon name ID selected for this type.

:``<polygon_name>``.type:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         N/A
  ============== =============================

  Description:
    Type of polygon shape. Available values are ``polygon``, ``circle``. Causes an error, if not specialized.

:``<polygon_name>``.points:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  vector<double> N/A
  ============== =============================

  Description:
    Polygon vertexes, listed in ``{p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, ...}`` format. Used for ``polygon`` type. Minimum 3 points for a triangle polygon. Causes an error, if not specialized.

:``<polygon_name>``.radius:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         N/A
  ============== =============================

  Description:
    Circle radius. Used for ``circle`` type. Causes an error, if not specialized.

:``<polygon_name>``.action_type:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         N/A
  ============== =============================

  Description:
    Safety behavior model. Available values are ``stop``, ``slowdown``, ``approach``. Causes an error, if not specialized.

:``<polygon_name>``.max_points:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  int            3
  ============== =============================

  Description:
    Maximum number of points to enter inside polygon to be ignored (w/o causing an action).

:``<polygon_name>``.slowdown_ratio:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.5
  ============== =============================

  Description:
    Robot slowdown (share of its actual speed). Applicable for ``slowdown`` action type.

:``<polygon_name>``.time_before_collision:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         2.0
  ============== =============================

  Description:
    Time before collision in seconds. Maximum simulation time used in collision prediction. Higher values mean lower performance. Applicable for ``approach`` action type.

:``<polygon_name>``.simulation_time_step:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.02
  ============== =============================

  Description:
    Time iteration step for robot movement simulation during collision prediction. Lower values mean lower prediction accuracy but better performance. Applicable for ``approach`` action type.

:``<polygon_name>``.visualize:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           False
  ============== =============================

  Description:
    Whether to publish the polygon in a separate topic.

:``<polygon_name>``.polygon_pub_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         <polygon_name>
  ============== =============================

  Description: 
    Topic name to publish a polygon to. Used only if ``visualize`` is true.

:``<polygon_name>``.footprint_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "footprint"
  ============== =============================

  Description:
    Topic to listen the robot footprint from. Applicable only for ``polygon`` type and ``approach`` action type.

Observation sources parameters
==============================

``<source name>`` is the corresponding data source name ID selected for this type.

:``<source name>``.type:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "scan"
  ============== =============================

  Description:
    Type of polygon shape. Could be ``scan`` or ``pointcloud``.

:``<source name>``.topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "scan"
  ============== =============================

  Description:
    Topic to listen the source data from.

:``<source name>``.min_height:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.05
  ============== =============================

  Description:
    Minimum height the PointCloud projection to 2D space started from. Applicable for ``pointcloud`` type.

:``<source name>``.max_height:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.5
  ============== =============================
  
  Description:
    Maximum height the PointCloud projection to 2D space ended with. Applicable for ``pointcloud`` type.

Example
*******

Here is an example of configuration YAML for the Collision Monitor.
For more information how to bring-up your own Collision Monitor node, please refer to the :ref:`collision_monitor_tutorial` tutorial.

.. code-block:: yaml

    collision_monitor:
      ros__parameters:
        base_frame_id: "base_footprint"
        odom_frame_id: "odom"
        cmd_vel_in_topic: "cmd_vel_raw"
        cmd_vel_out_topic: "cmd_vel"
        transform_tolerance: 0.5
        source_timeout: 5.0
        stop_pub_timeout: 2.0
        polygons: ["PolygonStop", "PolygonSlow", "FootprintApproach"]
        PolygonStop:
          type: "circle"
          radius: 0.3
          action_type: "stop"
          max_points: 3
          visualize: True
          polygon_pub_topic: "polygon_stop"
        PolygonSlow:
          type: "polygon"
          points: [1.0, 1.0, 1.0, -1.0, -0.5, -1.0, -0.5, 1.0]
          action_type: "slowdown"
          max_points: 3
          slowdown_ratio: 0.3
          visualize: True
          polygon_pub_topic: "polygon_slowdown"
        FootprintApproach:
          type: "polygon"
          action_type: "approach"
          footprint_topic: "/local_costmap/published_footprint"
          time_before_collision: 2.0
          simulation_time_step: 0.02
          max_points: 5
          visualize: False
        observation_sources: ["scan", "pointcloud"]
        scan:
          type: "scan"
          topic: "/scan"
        pointcloud:
          type: "pointcloud"
          topic: "/intel_realsense_r200_depth/points"
          min_height: 0.1
          max_height: 0.5
