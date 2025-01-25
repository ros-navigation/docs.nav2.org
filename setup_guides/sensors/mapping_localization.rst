Mapping and Localization
########################

Now that we have a robot with its sensors set up, we can use the obtained sensor information to build a map of the environment and to localize the robot on the map. The ``slam_toolbox`` package is a set of tools and capabilities for 2D Simultaneous Localization and Mapping (SLAM) in potentially massive maps with ROS2. It is also one of the officially supported SLAM libraries in Nav2, and we recommend to use this package in situations you need to use SLAM on your robot setup. Aside from the ``slam_toolbox``, localization can also be implemented through the ``nav2_amcl`` package. This package implements Adaptive Monte Carlo Localization (AMCL) which estimates the position and orientation of the robot in a map. Other techniques may also be available, please check Nav2 documentation for more information.

Both the ``slam_toolbox`` and ``nav2_amcl`` use information from the laser scan sensor to be able to perceive the robot's environment. Hence, to verify that they can access the laser scan sensor readings, we must make sure that they are subscribed to the correct topic that publishes the ``sensor_msgs/LaserScan`` message. This can be configured by setting their ``scan_topic`` parameters to the topic that publishes that message. It is a convention to publish the ``sensor_msgs/LaserScan`` messages to  ``/scan`` topic. Thus, by default, the ``scan_topic`` parameter is set to ``/scan``. Recall that when we added the lidar sensor to ``sam_bot`` in the previous section, we set the topic to which the lidar sensor will publish the ``sensor_msgs/LaserScan`` messages as ``/scan``. 

In-depth discussions on the complete configuration parameters will not be a scope of our tutorials since they can be pretty complex. Instead, we recommend you to have a look at their official documentation in the links below.

.. seealso::
  | For the complete list of configuration parameters of ``slam_toolbox``, see the `Github repository of slam_toolbox <https://github.com/SteveMacenski/slam_toolbox#readme>`_.
  | For the complete list of configuration parameters and example configuration of ``nav2_amcl``, see the `AMCL Configuration Guide <https://docs.nav2.org/configuration/packages/configuring-amcl.html>`_.

 
You can also refer to the `(SLAM) Navigating While Mapping guide <https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html>`_ for the tutorial on how to use Nav2 with SLAM. You can verify that ``slam_toolbox`` and ``nav2_amcl`` have been correctly setup by visualizing the map and the robot's pose in RViz, similar to what was shown in the previous section.


Costmap 2D
**********
The costmap 2D package makes use of the sensor information to provide a representation of the robot's environment in the form of an occupancy grid. The cells in the occupancy grid store cost values between 0-254 which denote a cost to travel through these zones. A cost of 0 means the cell is free while a cost of 254 means that the cell is lethally occupied. Values in between these extremes are used by navigation algorithms to steer your robot away from obstacles as a potential field. Costmaps in Nav2 are implemented through the ``nav2_costmap_2d`` package. 

The costmap implementation consists of multiple layers, each of which has a certain function that contributes to a cell's overall cost. The package consists of the following layers, but are plugin-based to allow customization and new layers to be used as well: static layer, inflation layer, range layer, obstacle layer, and voxel layer. The static layer represents the map section of the costmap, obtained from the messages published to the ``/map`` topic like those produced by SLAM.  The obstacle layer includes the objects detected by sensors that publish either or both the ``LaserScan`` and ``PointCloud2`` messages. The voxel layer is similar to the obstacle layer such that it can use either or both the ``LaserScan`` and ``PointCloud2`` sensor information but handles 3D data instead. The range layer allows for the inclusion of information provided by sonar and infrared sensors. Lastly, the inflation layer represents the added cost values around lethal obstacles such that our robot avoids navigating into obstacles due to the robot's geometry. In the next subsection of this tutorial, we will have some discussion about the basic configuration of the different layers in ``nav2_costmap_2d``. 

The layers are integrated into the costmap through a plugin interface and then inflated using a user-specified `inflation radius <http://wiki.ros.org/costmap_2d/hydro/inflation>`_, if the inflation layer is enabled. For a deeper discussion on costmap concepts, you can have a look at the `ROS1 costmap_2D documentation <http://wiki.ros.org/costmap_2d>`_. Note that the ``nav2_costmap_2d`` package is mostly a straightforward ROS2 port of the ROS1 navigation stack version with minor changes required for ROS2 support and some new layer plugins.

.. _configuring_nav2_costmap_2d:

Configuring nav2_costmap_2d
===========================
In this subsection, we will show an example configuration of ``nav2_costmap_2d`` such that it uses the information provided by the lidar sensor of ``sam_bot``. We will show an example configuration that uses static layer, obstacle layer, voxel layer, and inflation layer. We set both the obstacle and voxel layer to use the ``LaserScan`` messages published  to the ``/scan`` topic by the lidar sensor. We also set some of the basic parameters to define how the detected obstacles are reflected in the costmap. Note that this configuration is to be included in the configuration file of Nav2. 

.. code-block:: yaml
  :lineno-start: 1

  global_costmap:
    global_costmap:
      ros__parameters:
        update_frequency: 1.0
        publish_frequency: 1.0
        global_frame: map
        robot_base_frame: base_link
        use_sim_time: True
        robot_radius: 0.22
        resolution: 0.05
        track_unknown_space: false
        rolling_window: false
        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        static_layer:
          plugin: "nav2_costmap_2d::StaticLayer"
          map_subscribe_transient_local: True
        obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: True
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0
        inflation_layer:
          plugin: "nav2_costmap_2d::InflationLayer"
          cost_scaling_factor: 3.0
          inflation_radius: 0.55
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
        robot_radius: 0.22
        plugins: ["voxel_layer", "inflation_layer"]
        voxel_layer:
          plugin: "nav2_costmap_2d::VoxelLayer"
          enabled: True
          publish_voxel_map: True
          origin_z: 0.0
          z_resolution: 0.05
          z_voxels: 16
          max_obstacle_height: 2.0
          mark_threshold: 0
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
        inflation_layer:
          plugin: "nav2_costmap_2d::InflationLayer"
          cost_scaling_factor: 3.0
          inflation_radius: 0.55
        always_send_full_costmap: True

In the configuration above, notice that we set the parameters for two different costmaps: ``global_costmap`` and ``local_costmap``. We set up two costmaps since the ``global_costmap`` is mainly used for long-term planning over the whole map while ``local_costmap`` is for short-term planning and collision avoidance. 

The layers that we use for our configuration are defined in the ``plugins`` parameter, as shown in line 13 for the ``global_costmap`` and line 50 for the ``local_costmap``. These values are set as a list of mapped layer names that also serve as namespaces for the layer parameters we set up starting at lines 14 and line 51. Note that each layer/namespace in this list must have a ``plugin`` parameter (as indicated in lines 15, 18, 32, 52, and 68) defining the type of plugin to be loaded for that specific layer.

For the static layer (lines 14-16), we set the ``map_subscribe_transient_local`` parameter to ``True``. This sets the QoS settings for the map topic. Another important parameter for the static layer is the ``map_topic`` which defines the map topic to subscribe to. This defaults to ``/map`` topic when not defined. 

For the obstacle layer (lines 17-30), we define its sensor source under the ``observation_sources`` parameter (line 20) as ``scan`` whose parameters are set up in lines 22-30. We set its ``topic`` parameter as the topic that publishes the defined sensor source and we set the ``data_type`` according to the sensor source it will use. In our configuration, the obstacle layer will use the ``LaserScan`` published by the lidar sensor to ``/scan``. 

Note that the obstacle layer and voxel layer can use either or both ``LaserScan`` and ``PointCloud2`` as their ``data_type`` but it is set to ``LaserScan`` by default. The code snippet below shows an example of using both the ``LaserScan`` and ``PointCloud2`` as the sensor sources. This may be particularly useful when setting up your own physical robot.

.. code-block:: shell

  obstacle_layer:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    enabled: True
    observation_sources: scan pointcloud
    scan:
      topic: /scan
      data_type: "LaserScan"
    pointcloud:
      topic: /depth_camera/points
      data_type: "PointCloud2"

For the other parameters of the obstacle layer, the ``max_obstacle_height`` parameter sets the maximum height of the sensor reading to return to the occupancy grid. The minimum height of the sensor reading can also be set using the ``min_obstacle_height`` parameter, which defaults to 0 since we did not set it in the configuration. The ``clearing`` parameter is used to set whether the obstacle is to be removed from the costmap or not. The clearing operation is done by raytracing through the grid. The maximum and minimum range to raytrace clear objects from the costmap is set using the ``raytrace_max_range`` and ``raytrace_min_range`` respectively. The ``marking`` parameter is used to set whether the inserted obstacle is marked into the costmap or not. We also set the maximum and minimum range to mark obstacles in the costmap through the ``obstacle_max_range`` and ``obstacle_min_range`` respectively. 

For the inflation layer (lines 31-34 and 67-70), we set the exponential decay factor across the inflation radius using the ``cost_scaling_factor`` parameter. The value of the radius to inflate around lethal obstacles is defined using the ``inflation_radius``. 

For the voxel layer (lines 51-66), we set the ``publish_voxel_map`` parameter to ``True`` to enable the publishing of the 3D voxel grid. The resolution of the voxels in height is defined using the ``z_resolution`` parameter, while the number of voxels in each column is defined using the ``z_voxels`` parameter. The ``mark_threshold`` parameter sets the minimum number of voxels in a column to mark as occupied in the occupancy grid. We set the ``observation_sources`` parameter of the voxel layer to ``scan``, and we set the scan parameters (in lines 61-66) similar to the parameters that we have discussed for the obstacle layer. As defined in its ``topic`` and ``data_type`` parameters, the voxel layer will use the ``LaserScan`` published on the ``/scan`` topic by the lidar scanner.

Note that the we are not using a range layer for our configuration but it may be useful for your own robot setup. For the range layer, its basic parameters are the ``topics``, ``input_sensor_type``, and ``clear_on_max_reading`` parameters. The range topics to subscribe to are defined in the ``topics`` parameter. The ``input_sensor_type`` is set to either ``ALL``, ``VARIABLE``, or ``FIXED``. The ``clear_on_max_reading`` is a boolean parameter that sets whether to clear the sensor readings on max range.  Have a look at the configuration guide in the link below in case you need to set it up. 

.. seealso::
  For more information on ``nav2_costmap_2d`` and the complete list of layer plugin parameters, see the `Costmap 2D Configuration Guide <https://docs.nav2.org/configuration/packages/configuring-costmaps.html>`_.


Build, Run and Verification
===========================
We will first launch ``display.launch.py`` which launches the robot state publisher that provides the ``base_link`` => ``sensors`` transformations in our URDF, launches Gazebo that acts as our physics simulator, and provides the ``odom`` => ``base_link`` from the differential drive plugin or the ekf_node. It also launches RViz which we can use to visualize the robot and sensor information.

Then we will launch ``slam_toolbox`` to publish to ``/map`` topic and provide the ``map`` => ``odom`` transform. Recall that the ``map`` => ``odom`` transform is one of the primary requirements of the Nav2 system. The messages published on the ``/map`` topic will then be used by the static layer of the ``global_costmap``.

After we have properly setup our robot description, odometry sensors, and necessary transforms, we will finally launch the Nav2 system itself. For now, we will only be exploring the costmap generation system of Nav2. After launching Nav2, we will visualize the costmaps in RViz to confirm our output.

Launching Description Nodes, RViz and Gazebo
--------------------------------------------

Let us now launch our Robot Description Nodes, RViz and Gazebo through the launch file ``display.launch.py``. Open a new terminal and execute the lines below. 

.. code-block:: shell

  colcon build
  . install/setup.bash
  ros2 launch sam_bot_description display.launch.py

RViz and the Gazebo should now be launched with ``sam_bot`` present in both. Recall that the ``base_link`` => ``sensors`` transform is now being published by ``robot_state_publisher`` and the ``odom`` => ``base_link`` transform by our Gazebo plugins. Both transforms should now be displayed show without errors in RViz.

Launching slam_toolbox
----------------------

To be able to launch ``slam_toolbox``, make sure that you have installed the ``slam_toolbox`` package by executing the following command:

.. code-block:: shell

  sudo apt install ros-<ros2-distro>-slam-toolbox

We will launch the ``async_slam_toolbox_node`` of ``slam_toolbox`` using the package's built-in launch files. Open a new terminal and then execute the following lines:

.. code-block:: shell

  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

The ``slam_toolbox`` should now be publishing to the ``/map`` topic and providing the ``map`` => ``odom`` transform. 

We can verify in RViz that the ``/map`` topic is being published. In the RViz window, click the add button at the bottom-left part then go to ``By topic`` tab then select the ``Map`` under the ``/map`` topic. You should be able to visualize the message received in the ``/map`` as shown in the image below.

.. image:: images/map.png
    :align: center

We can also check that the transforms are correct by executing the following lines in a new terminal:

.. code-block:: shell

  ros2 run tf2_tools view_frames

The line above will create a ``frames.pdf`` file that shows the current transform tree. Your transform tree should be similar to the one shown below:

.. image:: images/view_frames.png
    :align: center

Launching Nav2
--------------
First, Make sure you have installed the Nav2 packages by executing the following:

.. code-block:: shell

  sudo apt install ros-<ros2-distro>-navigation2
  sudo apt install ros-<ros2-distro>-nav2-bringup

We will now launch Nav2 using the ``nav2_bringup``'s built-in launch file, ``navigation_launch.py`` . Open a new terminal and execute the following:

.. code-block:: shell

  ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

Note that the parameters of the ``nav2_costmap_2d`` that we discussed in the previous subsection are included in the default parameters of ``navigation_launch.py``. Aside from the ``nav2_costmap_2d`` parameters, it also contains parameters for the other nodes that are included in Nav2 implementation. 

After we have properly set up and launched Nav2, the ``/global_costmap`` and ``/local_costmap`` topics should now be active.

.. note::  
  To make the costmaps show up, run the 3 commands in this order

  #. Launching Description Nodes, RViz and Gazebo - wait a bit for everything to launch
  #. Launching slam_toolbox - in logs wait for "Registering sensor"
  #. Launching Nav2 - in logs wait for "Creating bond timer"

Visualizing Costmaps in RViz
----------------------------

The ``global_costmap``, ``local_costmap`` and the voxel representation of the detected obstacles can be visualized in RViz.

To visualize the ``global_costmap`` in RViz, click the add button at the bottom-left part of the RViz window. Go to ``By topic`` tab then select the ``Map`` under the ``/global_costmap/costmap`` topic. The ``global_costmap`` should show in the RViz window, as shown below. The ``global_costmap`` shows areas which should be avoided (black) by our robot when it navigates our simulated world in Gazebo.

.. image:: images/costmap_global_rviz.png
    :align: center

To visualize the ``local_costmap`` in RViz, select the ``Map`` under the ``/local_costmap/costmap`` topic. Set the ``color scheme`` in RViz to ``costmap`` to make it appear similar to the image below. 

.. image:: images/local_costmap_rviz.png
    :align: center

To visualize the voxel representation of the detected object, open a new terminal and execute the following lines:

.. code-block:: shell

  ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker

The line above sets the topic where the the markers will be published to ``/my_marker``. To see the markers in RViz, select ``Marker`` under the ``/my_marker`` topic, as shown below.

.. image:: images/add_my_marker.png
    :align: center
    :width: 49 % 

Then set the ``fixed frame`` in RViz to ``odom`` and you should now see the voxels in RViz, which represent the cube and the sphere that we have in the Gazebo world:

.. image:: images/voxel_layer.png
    :align: center

Conclusion
**********

In this section of our robot setup guide, we have discussed the importance of sensor information for different tasks associated with Nav2. More specifically, tasks such as mapping (SLAM), localization (AMCL), and perception (costmap) tasks. 
Then, we set up a basic configuration for the ``nav2_costmap_2d`` package using different layers to produce a global and local costmap. We then verify our work by visualizing these costmaps in RViz.
