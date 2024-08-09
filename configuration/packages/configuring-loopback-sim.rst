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
    The scan frame to use to publish a scan for collision monitor's happiness

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
