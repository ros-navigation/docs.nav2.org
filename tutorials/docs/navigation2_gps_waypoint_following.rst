.. _navigation2-gps-waypoint-following:

GPS Waypoint Following
************************

- `Overview`_
- `Tutorial Steps`_

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="660" height="371" src="https://www.youtube.com/embed/DQGfRRn1DBQ" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>   
      </div>             
    </h1>

Overview
========

This tutorial shows how to use Navigation2 for following GPS waypoints.

The requirements for this task are as follows:

- Collect GPS waypoints with orientation infromation.
- Setup `Robot Localization <https://github.com/cra-ros-pkg/robot_localization/>`_ such that utm -> map coordinate frame transform is available.
- Use provided sample action client code to request to action server for following of GPS waypoints.


Tutorial Steps
==============

0- Collect GPS Waypoints
------------------------

We can consider two cases here. GPS waypoint following is possible in both simulation and real. This tutorial was developed based on simulation but all the steps should also apply on a real system. 

Although It might not be the most convinient way, a practical way to collect the GPS waypoints is to traverse the path and collect GPS waypoints on certain points of the path and save the waypoint data to a yaml file.

The action client node then can read this waypoints in yaml file and go through them. Assuming that you have ``sensor_msgs::msg::NavSatFix`` message type available under /gps/fix topic and ``sensor_msgs::msg::Imu``  
message type available under /imu/data topic, you might drive the robot to points and use provided gps_waypoint_collectr node to colect GPS coordinates and robot's orientation at that point. 

We are interested in getting latitude, longitude and altitude at this location. We also need to consider a desired orientation at this pose. A global orientation can be acquired from an earth referenced IMU. Please see more details on
how the IMU should report the data at `Robot Localization <http://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html#imu>`_. For the orientation, only yaw angle is our concern here. 

We have provided a node in tutorial source code to collect synced imu and gps data at waypoints. The topics can be configured to your case. ;
``gps_waypoint_collector.launh.py`` in ``nav2_gps_waypoint_follower_demo`` ;

.. code-block:: python

  from launch import LaunchDescription
  import launch_ros.actions
  import os
  import yaml
  from launch.substitutions import EnvironmentVariable
  import pathlib
  import launch.actions
  from launch.actions import DeclareLaunchArgument
  from ament_index_python.packages import get_package_share_directory

  def generate_launch_description():
      return LaunchDescription([
          launch_ros.actions.Node(
                  package='nav2_gps_waypoint_follower_demo', 
                  executable='gps_waypoint_collector', 
                  name='gps_waypoint_collector_node',
                  output='screen',
                  remappings=[('/gps', '/gps/fix'),
                              ('/imu', '/imu/data')]
              )               
  ])

remmap your topics accordingly. 

You need to make sure that you have built packages in `navigation2_tutorials <https://github.com/ros-planning/navigation2_tutorials>`_ successfully. 
After you drove the robot to a waypoint then do ;

.. code-block:: bash
  source ~/your_colcon_ws/install/setup.bash
  ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_collector.launch.py

You should be able to see terminal outputting current GPS location and orientation in form of; 

.. code-block:: bash
  [gps_waypoint_collector-1] [INFO] [1609821278.276347895] [gps_waypoint_collector_node]: Entering to timer callback, this is periodicly called
  [gps_waypoint_collector-1] [INFO] [1609821278.276561015] [gps_waypoint_collector_node]: curr_gps_waypoint: [0.00000000, 0.00000003, 0.63917288, 0.7]
  [gps_waypoint_collector-1] [INFO] [1609821279.276582547] [gps_waypoint_collector_node]: curr_gps_waypoint: [0.00000000, 0.00000003, 0.63917288, 0.75]
  [gps_waypoint_collector-1] [INFO] [1609821280.276421593] [gps_waypoint_collector_node]: curr_gps_waypoint: [0.00000000, 0.00000003, 0.63917288, 0.755]

here the values corresponds to ; 

.. code-block:: bash
  curr_gps_waypoint: [lat, long, alt, yaw(radians)]  

This callback is called periodicly, if the robot moves the values will be updated, however it is reccomended that you stop at each waypoint then execute this node, get the latest message and save it in a yaml file.

Repeat this for each waypoint you would like to collect. Finally your yaml file with collected waypoints should look like soething similar to this;

.. code-block:: yaml
  gps_waypoint_follower_demo:
    ros__parameters:
      waypoints: [wp0,wp1,wp2,wp3,wp4]
      #lat, long, alt, yaw(radians)
      wp0: [9.677703999088216e-07, -5.306676831178058e-05, 0.6442248001694679 , 1.57]
      wp1: [9.677703999088216e-07, -5.306676831178058e-05, 0.6442248001694679 , 1.57]
      wp2: [4.169383611283205e-05, -0.0006143364570898212, 0.6346865268424153 , 0.0]
      wp3: [9.319715737387455e-05, -0.000620772355007051, 0.6348643703386188, 0.0]
      wp4: [8.37498018946476e-06, -2.402470336058297e-05, 0.6447164406999946, 3.14]
      .
      .

update the nav2_gps_waypoint_follower_demo/config/demo_gps_waypoints.yaml file with the point you have just collected. 

1- Configure Robot Localization
-------------------------------
WIP


2- Let The Robot Follow The GPS Waypoints
-----------------------------------------
WIP
