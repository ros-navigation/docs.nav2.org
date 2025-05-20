.. _configuring_loopback_sim:

Loopback Simulator
##################

Source code on Github_.

.. _Github: https://github.com/ros-navigation/navigation2/tree/main/nav2_loopback_sim


The ``nav2_loopback_sim`` is a stand-alone simulator to create a "loopback" for non-physical simulation to replace robot hardware, physics simulators (Gazebo, Bullet, Isaac Sim, etc).
It computes the robot's odometry based on the command velocity's output request to create a perfect 'frictionless plane'-style simulation for unit testing, system testing, R&D on higher level systems, testing behaviors without concerning yourself with localization accuracy or system dynamics, and multirobot simulations.

Parameters
**********

:update_duration:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.01
  ============== ==============

  Description
    The duration between updates (s)

:base_frame_id:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "base_link"
  ============== ==============

  Description
    The base frame to use.

:odom_frame_id:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "odom"
  ============== ==============

  Description
    The odom frame to use.

:map_frame_id:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "map"
  ============== ==============

  Description
    The map frame to use.

:scan_frame_id:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "base_scan"
  ============== ==============

  Description
    The scan frame to use to publish a scan

:enable_stamped_cmd_vel:

  ============== ==============
  Type           Default
  -------------- --------------
  string         True
  ============== ==============

  Description
    Whether cmd_vel is stamped or unstamped (i.e. Twist or TwistStamped).
    Note: This parameter is default ``false`` in Jazzy or older! Kilted or newer uses ``TwistStamped`` by default.

:scan_publish_dur:

  ============== ==============
  Type           Default
  -------------- --------------
  string         0.1
  ============== ==============

  Description
    The duration between publishing scan (in sec)

:publish_map_odom_tf:

  ============== ==============
  Type           Default
  -------------- --------------
  string         true
  ============== ==============

  Description
    Whether or not to publish tf from ``map_frame_id`` to ``odom_frame_id``

:publish_clock:

  ============== ==============
  Type           Default
  -------------- --------------
  string         true
  ============== ==============

  Description
    Whether or not to publish simulated clock to ``/clock``
  
:scan_range_min:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.05
  ============== ==============

  Description
    Minimum measurable distance from the scan in meters. Values below this are considered invalid

:scan_range_max:

  ============== ==============
  Type           Default
  -------------- --------------
  double         30.0
  ============== ==============

  Description
    Maximum measurable distance from the scan in meters. Values beyond this are out of range

:scan_angle_min:
  ============== ==============
  Type           Default
  -------------- --------------
  double         -3.14
  ============== ==============

  Description
     Starting angle of the scan in radians (leftmost angle)

:scan_angle_max: 
  ============== ==============
  Type           Default
  -------------- --------------
  double         3.14
  ============== ==============

  Description
    Ending angle of the scan in radians (rightmost angle)
:scan_angle_increment:
  ============== ==============
  Type           Default
  -------------- --------------
  double         0.0174
  ============== ==============

  Description
     Angular resolution of the scan in radians (angle between consecutive measurements)
:scan_use_inf:

  ============== ==============
  Type           Default
  -------------- --------------
  bool           true
  ============== ==============

  Description  
    Whether to use ``inf`` for out-of-range values.  
    If ``false``, values are set to ``scan_range_max - 0.1`` instead.

Example
*******
.. code-block:: yaml

    loopback_simulator:
      ros__parameters:
        base_frame_id: "base_footprint"
        odom_frame_id: "odom"
        map_frame_id: "map"
        scan_frame_id: "base_scan"  # tb4_loopback_simulator.launch.py remaps to 'rplidar_link'
        update_duration: 0.02
        scan_range_min: 0.05
        scan_range_max: 30.0
        scan_angle_min: -3.1415
        scan_angle_max: 3.1415
        scan_angle_increment: 0.02617
        scan_use_inf: true
