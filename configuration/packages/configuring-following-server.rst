.. _configuring_following_server:

Following Server
################

Source code on Github_.

.. _Github: https://github.com/ros-navigation/navigation2/nav2_following/opennav_following

The Following Server in ``opennav_following`` implements a server for following dynamic objects from a detection topic or specific reference frame.
This server allows the robot to follow and maintain a determined distance from a detected object or specific frame,
using topic-based detection techniques or coordinate frame tracking.
The server is designed to be called by a BT application or autonomy application to follow moving objects.

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700" height="450" src="https://www.youtube.com/embed/g-g58J1g9Ww?autoplay=1" frameborder="1" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
      </div>
    </h1>

Parameters
**********

:controller_frequency:

  ============== ==============
  Type           Default
  -------------- --------------
  double         50.0
  ============== ==============

  Description
    Control frequency (Hz) for the following control loop.

:detection_timeout:

  ============== ==============
  Type           Default
  -------------- --------------
  double         2.0
  ============== ==============

  Description
    Timeout (s) to wait for detection of the object to follow.

:rotate_to_object_timeout:

  ============== ==============
  Type           Default
  -------------- --------------
  double         10.0
  ============== ==============

  Description
    Timeout (s) to rotate searching for the object when detection is lost.

:static_object_timeout:

  ============== ==============
  Type           Default
  -------------- --------------
  double         -1.0
  ============== ==============

  Description
    Timeout (s) to stop following when the object remains static. If -1.0, the robot will follow indefinitely.

:linear_tolerance:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.15
  ============== ==============

  Description
    Linear tolerance (m) to consider that the target position has been reached.

:angular_tolerance:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.15
  ============== ==============

  Description
    Angular tolerance (rad) to consider that the target orientation has been reached.

:max_retries:

  ============== ==============
  Type           Default
  -------------- --------------
  int            3
  ============== ==============

  Description
    Maximum number of retries when detection or control fails.

:base_frame:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "base_link"
  ============== ==============

  Description
    Robot's base frame for control law.

:fixed_frame:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "odom"
  ============== ==============

  Description
    Fixed frame to use, recommended to be a smooth odometry frame **not** map.

:filter_coef:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.1
  ============== ==============

  Description
    Filter coefficient for smoothing object pose detections.

:desired_distance:

  ============== ==============
  Type           Default
  -------------- --------------
  double         1.0
  ============== ==============

  Description
    Desired distance (m) to maintain from the followed object.

:skip_orientation:

  ============== ==============
  Type           Default
  -------------- --------------
  bool           true
  ============== ==============

  Description
    If true, ignore the detected object's orientation and point toward it from the robot's position.

:search_by_rotating:

  ============== ==============
  Type           Default
  -------------- --------------
  bool           false
  ============== ==============

  Description
    If true, the robot will rotate in place when it loses object detection to try to find it again.

:search_angle:

  ============== ==============
  Type           Default
  -------------- --------------
  double         M_PI_2
  ============== ==============

  Description
    Maximum angle (rad) to rotate when searching for the object.

:odom_topic:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "odom"
  ============== ==============

  Description
    Odometry topic to use for obtaining the robot's current velocity.

:odom_duration:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         0.3
  ============== ===========================

  Description
    Time (s) to buffer odometry commands to estimate the robot speed.

:transform_tolerance:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.1
  ============== =============================

  Description
    Time with which to post-date the transform that is published, to indicate that this transform is valid into the future.

Example
*******
.. code-block:: yaml

    following_server:
      ros__parameters:
        controller_frequency: 50.0
        detection_timeout: 2.0
        rotate_to_object_timeout: 10.0
        static_object_timeout: 30.0  # -1.0 for indefinite following
        linear_tolerance: 0.15
        angular_tolerance: 0.15
        max_retries: 3
        base_frame: "base_link"
        fixed_frame: "odom"
        filter_coef: 0.1
        desired_distance: 1.0
        skip_orientation: true
        search_by_rotating: false
        odom_topic: "odom"
        odom_duration: 0.3
        transform_tolerance: 0.1

        # Controller configuration (inherited from docking controller)
        controller:
          k_phi: 3.0
          k_delta: 2.0
          beta: 0.4
          lambda: 2.0
          v_linear_min: 0.1
          v_linear_max: 0.5
          v_angular_max: 1.0
          slowdown_radius: 0.15
          use_collision_detection: false
          transform_tolerance: 0.1
