.. _setup_sensors_gz_classic:

Setting Up Sensors - Gazebo Classic
###################################

In this guide, we will discuss the importance of the sensors in navigating a robot safely and how to set up the sensors with Nav2. In the first half of this tutorial, we will take a brief look at commonly used sensors and common sensor messages in Nav2. Next, we will add a basic sensor setup on our previously built simulated robot, ``sam_bot``. Lastly, we will then verify the simulated sensor messages of ``sam_bot`` by visualizing them in RViz. 

Once sensors have been set up on a robot, their readings can be used in mapping, localization, and perception tasks. In the second half of this guide, we will first discuss how mapping and localization use the sensor data. Then, we will also take a look at one of Nav2's packages, ``nav2_costmap_2d``, which generates costmaps that will eventually be used in Nav2 path planning. We will set up basic configuration parameters for this package so it properly takes in sensor information from ``sam_bot``. Lastly, we visualize a generated costmaps in RViz to verify its received data.

Sensor Introduction
*******************
Mobile robots are equipped with a multitude of sensors that allow them to see and perceive their environment. These sensors obtain information which can be used to build and maintain the map of the environment, to localize the robot on the map, and to see the obstacles in the environment. These tasks are essential to be able to safely and efficiently navigate a robot through a dynamic environment. 

Examples of commonly used sensors are lidar, radar, RGB camera, depth camera, IMU, and GPS. To standardize the message formats of these sensors and allow for easier interoperation between vendors, ROS provides the ``sensor_msgs`` package that defines the common sensor interfaces. This also allows users to use any sensor vendor as long as it follows the standard format in ``sensor_msgs``. In the next subsection, we introduce some of commonly used messages in navigation, namely the ``sensor_msgs/LaserScan``, ``sensor_msgs/PointCloud2``, ``sensor_msgs/Range``, and ``sensor_msgs/Image``.

Aside from the ``sensor_msgs`` package, there are also the ``radar_msgs`` and ``vision_msgs`` standard interfaces you should be aware of.  The ``radar_msgs`` defines the messages for radar-specific sensors while the ``vision_msgs`` package defines the messages used in computer vision such as object detection, segmentation, and other machine learning models. Messages supported by this package are ``vision_msgs/Classification2D``, ``vision_msgs/Classification3D``, ``vision_msgs/Detection2D``, and ``vision_msgs/Detection3D``, to name a few.

.. seealso::
   For more information, see the API documentation of `sensor_msgs <http://wiki.ros.org/sensor_msgs>`_, `radar_msgs <http://wiki.ros.org/radar_msgs>`_, and `vision_msgs <http://wiki.ros.org/vision_msgs>`_.

Your physical robot's sensors probably have ROS drivers written for them (e.g. a ROS node that connects to the sensors, populates data into messages, and publishes them for your robot to use) that follow the standard interface in the ``sensor_msgs`` package. The ``sensor_msgs`` package makes it easy for you to use many different sensors from different manufacturers. General software packages like Nav2 can then read these standardized messages and perform tasks independent of the sensor hardware. On simulated robots such as ``sam_bot``, Gazebo has sensor plugins which also publish their information following the ``sensor_msgs`` package.

Common Sensor Messages
======================  

In this subsection, we discuss some of the common types of ``sensor_msgs`` you might encounter when setting up Nav2. We will provide a brief description for each sensor, an image of it being simulated in Gazebo and the corresponding visualization of the sensor readings in RViz.

.. note::  There are other types of ``sensor_msgs`` aside from the ones listed below.  The complete list of messages and their definitions can be found in the `sensor_msgs documentation <http://wiki.ros.org/sensor_msgs>`_.

sensor_msgs/LaserScan
---------------------

This message represents a single scan from a planar laser range-finder. This message is used in ``slam_toolbox`` and ``nav2_amcl`` for localization and mapping, or in ``nav2_costmap_2d`` for perception.

.. image:: images/sensor_laserscan.png

sensor_msgs/PointCloud2
-----------------------

This message holds a collection of 3D points, plus optional additional information about each point. This can be from a 3D lidar, a 2D lidar, a depth camera or more.

.. image:: images/sensor_pointcloud2.png

sensor_msgs/Range
-----------------

This is a single range reading from an active ranger that emits energy and reports one range reading that is valid along an arc at the distance measured. A sonar, IR sensor, or 1D range finder are examples of sensors that use this message.

.. image:: images/sensor_range.png

sensor_msgs/Image
-----------------

This represents the sensor readings from RGB or depth camera, corresponding to RGB or range values.

.. image:: images/sensor_image.png

Simulating Sensors using Gazebo Classic
***************************************
To give you a better grasp of how to set up sensors on a simulated robot, we will build up on our previous tutorials and attach sensors to our simulated robot ``sam_bot``. Similar to the previous tutorial where we used Gazebo plugins to add odometry sensors to ``sam_bot``, we will be using the Gazebo plugins to simulate a lidar sensor and a depth camera on ``sam_bot``. If you are working with a real robot, most of these steps are still required for setting up your URDF frames and it will not hurt to also add in the gazebo plugins for later use. 

To be able to follow the rest of this section, make sure that you have properly installed Gazebo. You can follow the instructions at the `Setup and Prerequisites <https://docs.nav2.org/setup_guides/odom/setup_odom_gz_classic.html#setup-and-prerequisites>`_ of the previous tutorial to setup Gazebo. 


Adding Gazebo Plugins to a URDF
===============================

Let us first add a lidar sensor to ``sam_bot``. Open the URDF file, `src/description/sam_bot_description.urdf <https://github.com/ros-navigation/navigation2_tutorials/blob/humble/sam_bot_description/src/description/sam_bot_description.urdf>`_ and paste the following lines before the ``</robot>`` tag.

.. code-block:: xml
  :lineno-start: 251

  <link name="lidar_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.125"/>
      <inertia ixx="0.001"  ixy="0"  ixz="0" iyy="0.001" iyz="0" izz="0.001" />
    </inertial>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
         <cylinder radius="0.0508" length="0.055"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
         <cylinder radius="0.0508" length="0.055"/>
      </geometry>
    </visual>
  </link>
    
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.12" rpy="0 0 0"/>
  </joint>
    
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>5</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.000000</resolution>
            <min_angle>0.000000</min_angle>
            <max_angle>6.280000</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.120000</min>
          <max>3.5</max>
          <resolution>0.015000</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="scan" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

In the code snippet above, we create a ``lidar_link`` which will be referenced by the ``gazebo_ros_ray_sensor`` plugin as the location to attach our sensor. We also set values to the simulated lidar's scan and range properties. Lastly, we set the ``/scan`` as the topic to which it will publish the ``sensor_msgs/LaserScan`` messages.

Next, let us add a depth camera to ``sam_bot``. Paste the following lines after the ``</gazebo>`` tag of the lidar sensor. 

.. code-block:: xml
  :lineno-start: 314

  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.015 0.130 0.022"/>
      </geometry>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.015 0.130 0.022"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.035"/>
      <inertia ixx="0.001"  ixy="0"  ixz="0" iyy="0.001" iyz="0" izz="0.001" />
    </inertial>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.215 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="camera_depth_frame"/>

  <joint name="camera_depth_joint" type="fixed">
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
    <parent link="camera_link"/>
    <child link="camera_depth_frame"/>
  </joint>

  <gazebo reference="camera_link">
    <sensor name="depth_camera" type="depth">
      <visualize>true</visualize>
      <update_rate>30.0</update_rate>
      <camera name="camera">
        <horizontal_fov>1.047198</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>3</far>
        </clip>
      </camera>
      <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
        <baseline>0.2</baseline>
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <frame_name>camera_depth_frame</frame_name>
        <pointCloudCutoff>0.5</pointCloudCutoff>
        <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
        <distortionK1>0</distortionK1>
        <distortionK2>0</distortionK2>
        <distortionK3>0</distortionK3>
        <distortionT1>0</distortionT1>
        <distortionT2>0</distortionT2>
        <CxPrime>0</CxPrime>
        <Cx>0</Cx>
        <Cy>0</Cy>
        <focalLength>0</focalLength>
        <hackBaseline>0</hackBaseline>
      </plugin>
    </sensor>
  </gazebo>

Similar to the lidar sensor, we create ``camera_link`` which will be referenced by the ``gazebo_ros_camera`` plugin as the sensor attachment location. We also create a ``camera_depth_frame`` that is attached to the ``camera_link`` and will be set as the ``<frame_name>`` of the depth camera plugin.  We also configure the plugin such that it will publish ``sensor_msgs/Image`` and ``sensor_msgs/PointCloud2`` messages to ``/depth_camera/image_raw`` and  ``/depth_camera/points`` topics respectively. Lastly, we also set up other basic configuration properties for our depth camera.

Launch and Build Files
======================

To verify that the sensors are set up properly and that they can see objects in our environment, let us launch ``sam_bot`` in a Gazebo world with objects. 
Let us create a Gazebo world with a single cube and a single sphere that are within the range of ``sam_bot``'s sensors so we can verify if it can see the objects correctly. 

To create the world, create a directory named ``world`` at the root of your project and create a file named ``my_world.sdf`` inside the ``world`` folder . Then copy the contents of `world/my_world.sdf <https://github.com/ros-navigation/navigation2_tutorials/blob/humble/sam_bot_description/world/my_world.sdf>`_ and paste them inside ``my_world.sdf``.

Now, let us edit our launch file, `launch/display.launch.py <https://github.com/ros-navigation/navigation2_tutorials/blob/humble/sam_bot_description/launch/display.launch.py>`_, to launch Gazebo with the world we just created. First, add the path of ``my_world.sdf`` by adding the following lines inside the ``generate_launch_description()``:

.. code-block:: shell

  world_path=os.path.join(pkg_share, 'world/my_world.sdf')

Lastly, add the world path in the ``launch.actions.ExecuteProcess(cmd=['gazebo',...`` line, as shown below.

.. code-block:: shell

  launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

We also have to add the ``world`` directory to our ``CMakeLists.txt`` file. Open `CmakeLists.txt <https://github.com/ros-navigation/navigation2_tutorials/blob/humble/sam_bot_description/CMakeLists.txt>`_ and append the ``world`` directory inside the install(DIRECTORY...), as shown in the snippet below.

.. code-block:: shell

  install(
    DIRECTORY src launch rviz config world
    DESTINATION share/${PROJECT_NAME}
  )

Build, Run and Verification
===========================

We can now build and run our project. Navigate to the root of the project and execute the following lines:

.. code-block:: shell

  colcon build
  . install/setup.bash
  ros2 launch sam_bot_description display.launch.py

RViz and the Gazebo will then be launched with ``sam_bot`` present in both. In the Gazebo window, the world that we created should be launched and ``sam_bot`` should be spawned in that world. You should now be able to observe ``sam_bot`` with the 360 lidar sensor and the depth camera, as shown in the image below.

.. image:: images/gazebo_sensors.png
    :align: center

In the RViz window, we can verify if we have properly modeled our sensors and if the transforms of our newly added sensors are correct:

.. image:: images/rviz_sensors.png
    :align: center

Lastly, we can also visualize the sensor readings in RViz.  To visualize the ``sensor_msgs/LaserScan`` message published on ``/scan`` topic, click the add button at the bottom part of the RViz window. Then go to the ``By topic`` tab and select the ``LaserScan`` option under ``/scan``, as shown below.

.. image:: images/add_topic_laserscan.png
    :align: center
    :width: 400

Next, set the ``Reliability Policy`` in RViz to ``Best Effort`` and set the ``size`` to 0.1 to see the points clearer. You should see the visualized ``LaserScan`` detection as shown below. This corresponds to the detected cube and sphere that we added to the Gazebo world. 

.. image:: images/demo_laserscan_rviz.png
    :align: center

To visualize ``sensor_msgs/Image`` and ``sensor_msgs/PointCloud2``, do the same for topics ``/depth_camera/image_raw`` and ``/depth_camera/points`` respectively:

.. image:: images/add_topic_image_pointcloud2.png

After adding the ``/depth_camera/image_raw`` topic in RViz, set the ``Reliability Policy`` in RViz to ``Best Effort``. Then you should see the cube in the image window at the lower-left side of the RViz window, as shown below.

.. image:: images/demo_image_rviz.png
    :align: center

You should also see the ``sensor_msgs/PointCloud2``, as shown below.

.. image:: images/pointcloud2_data.png
    :align: center


Conclusion
**********

In this section of our robot setup guide, we had a discussion on the common types of sensor messages in Nav2 which standardize the message formats for different sensor vendors. We also discussed how to add sensors to a simulated robot using Gazebo and how to verify that the sensors are working correctly through RViz. 
