.. _navigation2-gps-navigation:

GPS navigation
**************

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

This tutorial shows you how to use Navigation2 for GPS Navigation, the tutorial includes steps to do a GPS waypoint following.
Follow each step carefully.

The requirements for this task are as follows:

- Collect GPS waypoints with orientation information.
- Setup `Robot Localization <https://github.com/cra-ros-pkg/robot_localization/>`_ such that utm -> map coordinate frame transform is available.
- Use provided sample action client code to request to action server for following of GPS waypoints.

Tutorial Steps
==============

0- Collect GPS Waypoints
------------------------

We can consider two cases here. GPS waypoint following is possible in both simulation and real. This tutorial was developed based on simulation but all the steps should also apply on a real system. 

Although It might not be the most convinient way, a practical way to collect the GPS waypoints is to traverse the path and collect GPS waypoints on certain points of the path and save the waypoints to a yaml file.

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

remap your topics accordingly. 

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

Robot Localization is at core of GPS navigation. We rely on tools here to transform GPS latitude and longitude to map frame, which is the frame robot can perform navigation within. 

GPS data is not continous and it is subject to "jumps". This can be issue for planning and navigation in general, Robot Localization tries to deal with this by using two Extended Kalman 
Filter(EKF) nodes, the first node relies on continous sensor data such as IMU and Wheel odometry to construct a "local" 
pose estimation for robot. The second node includes GPS data, and fuses this GPS data with the results of first EKF node("local") pose estimation which results in a "global" pose estimation. 

The results of local pose estimation is subject to shifts over time due to wheel slips and integration errors. However the global pose estimaion is consistent against translation shifts over time.

Robot Localization provides a dedicated node(`navsat_transform_node`) to convert GPS coordinates to odometry messages, the frame for this odometry is located at the start pose of robot.  

More details on integrating GPS data can also be found at `here <http://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html>`_. 

The mentioned 3 nodes above should be configured with the right parameters. The below configuration was tested in simulation. The main changes should be made to topic names. The description of each indvidual
parameter is again available at Robot Localization wiki page for EKF node details.

.. code-block:: yaml
  # This is configuration for local pose estmation EKF node
  ekf_local_filter_node:
    ros__parameters:
      use_sim_time: true
      clear_params: true
      publish_tf: true
      filter_type: "ekf"
      frequency: 30.0
      sensor_timeout: 0.1
      odom0: /odometry/wheel                  # channge this according to your odometry source
      imu0: /imu/data                         # your imu topic
      odom_frame: odom                        # odometry frame
      base_link_frame: base_link
      world_frame: odom
      map_frame: map
      odom0_config: [false,  false, false, # X , Y , Z
                      false, false, false, # roll , pitch ,yaw
                      true,  true,  true,  # dX , dY , dZ
                      false, false, false, # droll , dpitch ,dyaw
                      false, false, false] # ddX , ddY , ddZ
      odom0_relative: false
      odom0_differential: false
      odom0_queue_size: 10
      imu0_config: [false,  false, false,  # X , Y , Z
                    false,  false,  true,  # roll , pitch ,yaw
                    false,  false, false,  # dX , dY , dZ
                    false,  false,  true,  # droll , dpitch ,dyaw
                    false,  false,  false] # ddX , ddY , ddZ
      imu0_relative: false                
      imu0_differential: false
      imu0_queue_size: 10
      imu0_remove_gravitational_acceleration: true
      process_noise_covariance: [0.03, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.03, 0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.04, 0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.03, 0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.03, 0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.0,    0.06, 0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.025, 0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.025, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.05, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.002, 0.0,    0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.002, 0.0,    0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.004, 0.0,    0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.01, 0.0,    0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.01, 0.0,
                                  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.01]
  # This is configuration for global pose estmation EKF node
  ekf_global_filter_node:
    ros__parameters:
      use_sim_time: true
      clear_params: true
      publish_tf: true
      filter_type: "ekf"
      frequency: 30.0
      sensor_timeout: 0.1
      odom0: /odometry/wheel
      odom1: /odometry/gps             # attention to this, this is coming from below node: navsat_transform_node
      imu0: /imu/data
      map_frame: map
      odom_frame: odom
      base_link_frame: base_link
      world_frame: map                 # we set world frame o map here, menaing that globl frmae will be map
      odom0_config: [false,  false,  false, # X , Y , Z
                      false, false, false,  # roll , pitch ,yaw
                      true, true, true,     # dX , dY , dZ
                      false, false, true,   # droll , dpitch ,dyaw
                      false, false, false]  # ddX , ddY , ddZ
      odom0_relative: false
      odom0_differential: false
      odom0_queue_size: 10
      odom1_config: [true,  true,  false, # X , Y , Z
                    false, false, false, # roll , pitch ,yaw
                    false, false, false, # dX , dY , dZ
                    false, false, false,  # droll , dpitch ,dyaw
                    false, false, false] # ddX , ddY , ddZ
      odom1_relative: false
      odom1_differential: false
      odom1_queue_size: 10
      imu0_config: [false,  false, false,  # X , Y , Z
                    false,  false,  true,  # roll , pitch ,yaw
                    false,  false, false,  # dX , dY , dZ
                    false,  false,  true,  # droll , dpitch ,dyaw
                    false,  false,  false] # ddX , ddY , ddZ
      imu0_relative: false
      imu0_differential: false
      imu0_queue_size: 10
      imu0_remove_gravitational_acceleration: true
      process_noise_covariance: [0.05, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                0.0,    0.05, 0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                0.0,    0.0,    0.06, 0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                0.0,    0.0,    0.0,    0.03, 0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                0.0,    0.0,    0.0,    0.0,    0.03, 0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                0.0,    0.0,    0.0,    0.0,    0.0,    0.06, 0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.025, 0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.025, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.04, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.01, 0.0,    0.0,    0.0,    0.0,    0.0,
                                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.01, 0.0,    0.0,    0.0,    0.0,
                                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.02, 0.0,    0.0,    0.0,
                                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.01, 0.0,    0.0,
                                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.01, 0.0,
                                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.015]      

  navsat_transform_node:
    ros__parameters:
      frequency: 10.0
      delay: 1.0
      magnetic_declination_radians: 0.0   
      yaw_offset: 0.0  
      zero_altitude: false
      publish_filtered_gps: true
      use_odometry_yaw: true
      broadcast_utm_transform: true
      broadcast_utm_transform_as_parent_frame: true # this is required when we convert GPS waypoint to map frame

Now that we have configuration for our nodes, next we should have a launch file to start Robot Localization nodes so that we have a tf tree with as utm -> map -> odom -> base_link -> .... 

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

      nav2_gps_waypoint_follower_demo_dir = get_package_share_directory('nav2_gps_waypoint_follower_demo')
      parameters_file_dir = os.path.join(nav2_gps_waypoint_follower_demo_dir, 'params')
      parameters_file_path = os.path.join(parameters_file_dir, 'dual_ekf_navsat_localization.yaml')
      return LaunchDescription([

          launch_ros.actions.Node(
                  package='robot_localization', 
                  executable='ekf_node', 
                  name='ekf_local_filter_node',
                  output='screen',
                  parameters=[parameters_file_path],
                  remappings=[('odometry/filtered', 'odometry/local')]           
                ),
          launch_ros.actions.Node(
                  package='robot_localization', 
                  executable='ekf_node', 
                  name='ekf_global_filter_node',
                  output='screen',
                  parameters=[parameters_file_path],
                  remappings=[('odometry/filtered', 'odometry/global')]
                ),           
          launch_ros.actions.Node(
                  package='robot_localization', 
                  executable='navsat_transform_node', 
                  name='navsat_transform_node',
                  output='screen',
                  parameters=[parameters_file_path],
                  remappings=[('odometry/filtered', 'odometry/global')]           
                )           
  ])

Please watch out for topic names. Configure right topics for your sensor data in the `dual_ekf_navsat_localization.yaml`.
The configuration file and launch file are also available in `navigation2_tutorials/nav2_gps_waypoint_follower_demo <https://github.com/ros-planning/navigation2_tutorials>`_. 
 
You can initialize the robot localization nodes after you have made right changes fo your sensor data. 

You need to make sure that you have built packages in `navigation2_tutorials <https://github.com/ros-planning/navigation2_tutorials>`_ successfully. 
After sucessful build do;

.. code-block:: bash
  source ~/your_colcon_ws/install/setup.bash
  ros2 launch nav2_gps_waypoint_follower_demo dual_ekf_navsat_localization.launch.py

The utm -> map -> odom -> base_link chain should now be available in the TF tree. 

2- Important navigation2 parameters that effect GPS Navigation
--------------------------------------------------------------

Normally navigation2 expects you to provide a static map, so nav2_map_server can read up that map and include that information in global costmap.
Of course it is still possible to map an outdoor Environment but for this tutorial we do not consider SLAM. We assume that the localization is made availbe with GPS,
see Robot Localization step of this tutorial.

We still need global and local costmaps. These two costmaps basically relies on live sensory data for obstacle free navigation. 
You might have a pointCloud or laserScan. There are a few important parameters that needs to be setup correctly for costmaps. 

The costmaps should be rolling_window enabled, so that they could `roll` together with robot. 

.. code-block:: python
  rolling_window: true

The planner cannot provide a plan that exceeds boundry of global costmap, therefore you need to set a reasonable size for your global costmap so that planner can handle requests accordingly.
For example with adding following configuration, planner will be able to handle goals that are up to 80 metres far away.
.. code-block:: python
  width: 80
  height: 80
  resolution: 0.2

Adjusts the sizes of global and local costmaps according to your desired reachability.   


3- Let The Robot Follow The GPS Waypoints
-----------------------------------------

Assuming that you have already collected your waypoints and they reside at `navigation2_tutorials/nav2_gps_waypoint_follower_demo/params/demo_gps_waypoints.yaml`. 

You can simply launch the GPS waypoint following with provided node. 

You need to make sure that you have built packages in `navigation2_tutorials <https://github.com/ros-planning/navigation2_tutorials>`_ successfully. 
After sucessful build do;

.. code-block:: bash
  source ~/your_colcon_ws/install/setup.bash
  ros2 launch nav2_gps_waypoint_follower_demo demo_gps_waypoint_follower.launch.py

The launch file looks something like ; 
 
.. code-block:: python

  from ament_index_python.packages import get_package_share_directory

  from launch import LaunchDescription
  from launch_ros.actions import LifecycleNode
  from launch.actions import DeclareLaunchArgument
  from launch.substitutions import LaunchConfiguration
  from launch.actions import EmitEvent
  from launch.actions import RegisterEventHandler
  from launch_ros.events.lifecycle import ChangeState
  from launch_ros.events.lifecycle import matches_node_name
  from launch_ros.event_handlers import OnStateTransition
  from launch.actions import LogInfo
  from launch.events import matches_action
  from launch.event_handlers.on_shutdown import OnShutdown

  import lifecycle_msgs.msg
  import os


  def generate_launch_description():
      share_dir = get_package_share_directory(
          'nav2_gps_waypoint_follower_demo')
      parameter_file = LaunchConfiguration('params_file')
      node_name = 'gps_waypoint_follower_demo'

      params_declare = DeclareLaunchArgument('params_file',
                                            default_value=os.path.join(
                                                share_dir, 'params', 'demo_gps_waypoints.yaml'),
                                            description='FPath to the ROS2 parameters file to use.')

      driver_node = LifecycleNode(package='nav2_gps_waypoint_follower_demo',
                                  executable='gps_waypoint_follower_demo',
                                  name=node_name,
                                  namespace='',
                                  output='screen',
                                  parameters=[parameter_file],
                                  )

      return LaunchDescription([
          params_declare,
          driver_node,
      ])

If you save more than one different waypoint data files then just change the demo_gps_waypoints.yaml with your desired file. 