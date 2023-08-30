.. _navigation2-with-gps:

Navigating with a GPS Based Localization System
***********************************************

- `Overview`_
- `Requirements`_
- `GPS Localization Overview`_
- `Tutorial Steps`_

.. image:: images/Gps_Navigation/interactive_wpf.gif
  :width: 600px
  :align: center

Overview
========

This tutorial shows how to set up a localization system using a GPS sensor as source of global positioning and robot_localization (RL) for sensor fusion, and how to use Nav2 to follow GPS waypoints. It was written by Pedro Gonzalez at `Kiwibot <https://www.kiwibot.com/>`_

Requirements
============

It is assumed ROS2 and Nav2 dependent packages are installed or built locally. Additionally you will have to install robot_localization and mapviz: 

   .. code-block:: bash

      sudo apt install ros-<ros2-distro>-robot-localization
      sudo apt install ros-<ros2-distro>-mapviz
      sudo apt install ros-<ros2-distro>-mapviz-plugins
      sudo apt install ros-<ros2-distro>-tile-map
    
The code for this tutorial is hosted on `nav2_gps_waypoint_follower_demo <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo>`_. Though we will go through the most important steps of the setup, it's highly recommended that you clone and build the package when setting up your dev environment.

GPS Localization Overview
=========================

GPS (Global Positioning System) or more broadly GNSS (Global Navigation Satellite System) is a technology that relies on satellites to provide receivers with an estimate of where they are located on the earth. These satellites are in orbit at altitudes around 20.000km and use radio frequency to continuously broadcast time signals, these are picked up by receivers when satellites are along their line of sight, they use trilateration to estimate their latitude, longitude and altitude.

GPS devices calculate their position using the `WGS84 standard <https://en.wikipedia.org/wiki/World_Geodetic_System>`_, which defines a cartesian system the earth's origin on its center of mass, the `z` axis pointing north and the `x` axis pointing to the first meridian as the image below shows.

.. image:: images/Gps_Navigation/WGS_84_reference_frame.svg
    :width: 562px
    :align: center
    :alt: WGS84 reference frame

However, this reference system is impractical for describing the motion of objects in or close to the earth's surface: Imagine your robot is located on a soccer field and you want it to move from one end to the other, your navigation task would look something like:

  "go from X=4789.413km, Y=177.511km z=4194.292km to X=4789.475km, Y=177.553km z=4194.22km"
  
It would make much more sense to create a local reference system where you could tell your robot "go 100 meters forward", right?

To cope with this, geodesy allows for the definition of  `Local tangent planes <https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates>`_. According to `REP 103 <https://www.ros.org/reps/rep-0103.html>`_ the coordinate systems of these planes should follow the ENU convention, which means their +x axis should face East, +y axis should face North and the +z axis should point Up, as the image below shows. In the context of this tutorial the GPS coordinates of the origin of the local cartesian plane will be called **datum** as it is in `robot_localization <http://docs.ros.org/en/noetic/api/robot_localization/html/index.html>`_.

.. image:: images/Gps_Navigation/ECEF_ENU_Longitude_Latitude_relationships.svg
    :width: 520px
    :align: center
    :alt: Local tangent plane


In this tutorial we assume the robot's GPS produces a really accurate and smooth estimation of the robot's position, however in the real world for standalone GPSs that's often not the case: you should expect accuracies of 1-2 meters under excellent conditions and up to 10 meters, and frequent jumps in the position as the GPS sensor picks up less or more satellites.

Several positioning augmentation technologies exists to reduce the error of GPS measurements, one of the most common ones is called `RTK <https://en.wikipedia.org/wiki/Real-time_kinematic_positioning>`_ (Real Time Kinematic Positioning), which can bring the accuracy of receivers down to 1cm. If Accuracy matters in your application this technology is highly recommended; though this requires the deployment of a second fixed GPS called Base, most of the US and Europe are already covered with public free to use Bases that you can connect to. You can read more about RTK and how to get started on `this tutorial <https://learn.sparkfun.com/tutorials/setting-up-a-rover-base-rtk-system>`_.

Additionally, to fully describe a robot's localization using the ENU convention we need to know its heading as well, however GPS sensors do not provide orientation measurements, only position measurements. Though there are other alternatives for measuring orientation, in this tutorial we assume the robot is equipped with an IMU capable of providing absolute heading measurements, meaning it will output zero yaw when facing east and +90 degrees when facing north. 

Despite the above assumption, in the real world commercial grade IMU's mounted in actual robots will often not produce accurate measurements of absolute heading because: 

1. They are hard to calibrate: outdoors robots are often big and heavy: imagine doing an eight figure in the air with an autonomous tractor.

2. They rely on magnetometers to measure earth's magnetic field but robots are often a huge source of electromagnetic noise: Electric motors are full of permanent magnets and can draw several amps, producing significant disturbances to the sensor.

Through the development of the tutorial we will see how to leverage robot_localization's Kalman Filters to mitigate this problem.

Tutorial Steps
==============

0- Setup Gazebo World
---------------------

To navigate using GPS we first need to create an outdoors Gazebo world with a robot having a GPS sensor. For this tutorial we will be using the `sonoma raceway <https://docs.px4.io/main/en/sim_gazebo_classic/gazebo_worlds.html#sonoma-raceway>`_ because its aligned with the real location. A sample world has been setup `here <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/worlds/sonoma_raceway.world>`_ using gazebo's spherical coordinates plugin, which creates a local tangent plane centered in the set geographic origin and provides latitude, longitude and altitude coordinates for each point in the world.

To get actual gps readings we need to create a robot model with a GPS sensor. An updated turtle model is provided in the `tutorial repo <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/models/turtlebot_waffle_gps>`_, which uses gazebo_ros's gps sensor plugin to output ``NavSatFix`` messages on the topic ``/gps/fix``.

Build the nav2_gps_waypoint_follower_demo package, source your workspace and test your gazebo world is properly set up by launching: 

  ``ros2 launch nav2_gps_waypoint_follower_demo gazebo_gps_world.launch.py``

A turtlebot waffle should appear in the sonoma raceway world. You may also echo the topic ``/gps/fix`` to verify the robot is indeed producing gps measurements 

.. image:: images/Gps_Navigation/gazebo_sonoma_raceway.png
    :width: 700px
    :align: center
    :alt: Turtlebot in the sonoma raceway
 
1- Setup GPS Localization system
--------------------------------

Once you have your simulation (or real robot) up and running, it's time to set up your localization system. Remember that nav2 needs a ``tf`` chain with the structure ``map`` -> ``odom`` -> ``base_link`` -> ``[sensor frames]``; global localization (``map`` -> ``base_link``) is usually provided by ``amcl``, while local odometry (``odom`` -> ``base_link``) is usually provided by the user's odometry system (wheel odometry, visual odometry, etc).

In this tutorial, the GPS sensor on the robot will replace ``amcl`` in providing global localization. Though you may build a custom module that takes in the ``NavSatFix`` and ``Imu`` messages of your GPS and imu, and outputs a ``tf`` between your ``map`` and ``base_link`` frames using a local tangent plane, nav2's gps waypoint follower needs robot_localization to be used for this purpose. This package already has a node that performs the GPS -> local cartesian conversions called the `navsat_transform_node <http://docs.ros.org/en/jade/api/robot_localization/html/navsat_transform_node.html>`_, and features state estimation nodes that use Kalman Filters to fuse multiple sources of data.

We will setup one extended kalman filter for local odometry, fusing wheel odometry and IMU data; and a second one for global localization, fusing the local cartesian coverted GPS coordinates, the wheel odometry and the IMU data. This is a common setup on robot_localization when using GPS data and more details around its configuration can be found in `RL's docs <http://docs.ros.org/en/jade/api/robot_localization/html/integrating_gps.html>`_. 

A `configuration file <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/config/dual_ekf_navsat_params.yaml>`_ and a `launch file <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/launch/dual_ekf_navsat.launch.py>`_ are provided for this purpose. You may take a while before continuing to understand these two files and what they configure. Note that the EKFs are set to work in 2D mode. This is because nav2's costmap environment representation is 2-Dimensional, and several layers rely on the ``base_link`` frame being on the same plane as their global frame (``map`` or ``odom`` if global or local costmap) for the height related parameters to make sense.

The navsat transform node exposes the ``datum`` parameter to set the GPS coordinates of the origin of the local tangent plane, which given translates to the origin of the position measurements output through its odometry output (``odom1`` in RL's params). The node will set this automatically to the coordinates of the first valid `NavSatFix` message it gets, however you may specify them said parameter in the yaml file or calling the ``/datum`` service in runtime. In this tutorial we will go with the automatic initialization because there is no information about the environment stored in cartesian coordinates (a static map, semantic navigation waypoints, etc), however if that's the case in your application you may fix the ``datum`` so a given pair of coordinates produced by the GPS always correspond to the same cartesian coordinates in your reference system.

A `static transforms launch <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/launch/static_transforms.launch.py>`_ is provided to set the transforms between ``base_link`` and all the robot sensors for nav2 and robot_localization.

As a sanity check that everything is working correctly, launch RL's launch file while gazebo is still running: 

  ``ros2 launch nav2_gps_waypoint_follower_demo dual_ekf_navsat.launch.py``

On a different terminal launch mapviz using the pre-built `config file <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/config/gps_wpf_demo.mvc>`_ in the repo. `Get a bing maps API key <https://www.microsoft.com/en-us/maps/create-a-bing-maps-key>`_ and use it to display satellite pictures.

  ``ros2 launch nav2_gps_waypoint_follower_demo mapviz.launch.py``

And finally run the turtle teleop key node to teleoperate the simulated turtlebot: 

  ``ros2 run teleop_twist_keyboard teleop_twist_keyboard``

When you have everything up and running, start teleoperating the turtlebot and check that:

1. When the robot faces east (default initial heading) and you move it forward, the ``base_link`` frame (green arrow) moves east consistently with the raw GPS measurements (blue dot).

2. Movement is consistent overall not only when facing east, meaning that the GPS measurements are consistent with the robot heading and movement direction, and that they are consistent with the position of the robot in the world (for instance, when the robot moves towards the finish line, GPS measurements in mapviz do as well).

The gif below shows what you should see:

.. image:: images/Gps_Navigation/localization_check.gif
  :width: 600px
  :align: center

Sensors in a real robot may be less accurate than gazebo's, specially GPSs and absolute heading measurements from IMUs. To mitigate this you can leverage robot_localization's EKFs to complement sensor's capabilities:

1. If your IMU does not provide absolute heading measurements accurately, consider setting the ``differential`` parameter of its input to RL to ``true``. This way the filter will only fuse changes in the orientation and derive the absolute value from its motion model, internally differentiating changes in the absolute position to estimate where the robot was heading.

2. If your GPS is noisy but you have a trustworthy wheel odometry source, consider tuning the sensors and process noise covariances to make the filter "trust" more or less one data source or its own internal state estimate. A properly tuned filter should be able to reject wrong GPS measurements to some degree.


2- Setup Navigation system
--------------------------

Once you have your localization system up and running it's time to setup nav2. since RL is already providing the ``tf`` tree we don't need to launch ``amcl``, thus we can remove its parameters from the params file.

Outdoors environments can get quite big, to a degree that they may not me practically represented on a single costmap. For that reason in this tutorial we use a rolling global costmap that is big enough for fitting successive pairs of waypoints, however depending on your application you may still choose to use a fixed global costmap, just remember to make it fit all the potential locations the robot may visit.

Additionally we assume that there's no static map of the environment, however if you decide to use one make sure to maintain consistency with GPS coordinates by selecting a fixed datum.

We provide a `nav2 params file <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/config/nav2_no_map_params.yaml>`_ and a `launch file <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/gps_waypoint_follower.launch.py>`_ to put it all together. Remember that the GPS setup of robot_localization was just a mean for setting up the global localization system, however nav2 is still a cartesian navigation stack and you may still use all its cartesian tools. To confirm that everything is working, launch the provided file (this launches gazebo and RL as well so close them if you have them running from the previous steps) and use rviz to send a goal to the robot:

  ``ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_follower.launch.py``

The gif below shows what you should see

.. image:: images/Gps_Navigation/navigation_check.gif
  :width: 600px
  :align: center

3-  Interactive GPS Waypoint Follower
-------------------------------------

Now that we have performed our complete system setup, lets leverage nav2 GPS waypoint follower capabilities to navigate to goals that are expressed directly in GPS coordinates. For this demo we want to build an interactive interface similar to rviz's that allows us to click over a map to make the robot navigate to the clicked location. For that we will use mapviz's point click publisher on the ``wgs84`` reference frame, which will publish a ``PointStamped`` message with the GPS coordinates of the point clicked over the satellite image.

For this purpose we provide the `interactive_waypoint_follower <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/nav2_gps_waypoint_follower_demo/interactive_waypoint_follower.py>`_ python node, which subscribes to mapviz's topic and calls the ``/follow_gps_waypoints`` action server with the clicked point as goal using the ``BasicNavigator`` in ``nav2_simple_commander``. To run it source your workspace and with the rest of the system running type:

  ``ros2 run nav2_gps_waypoint_follower_demo interactive_waypoint_follower``

Then launch mapviz using the pre-built config:

  ``ros2 launch nav2_gps_waypoint_follower_demo mapviz.launch.py``

You can now click on the mapviz map the pose you want the robot to go. The gif below shows the robot navigating to the finish line going through some obstacles:

.. image:: images/Gps_Navigation/interactive_wpf.gif
  :width: 600px
  :align: center

4-  Logged GPS Waypoint Follower
--------------------------------

Finally let's make a robot go through a set of predefined GPS waypoints stored in a yaml file. For this purpose we provide the `logged_waypoint_follower <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/nav2_gps_waypoint_follower_demo/logged_waypoint_follower.py>`_ node and a `waypoints <https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo/config/demo_waypoints.yaml>`_ file. To test this node source your workspace and with the rest of the system running type:

  ``ros2 run nav2_gps_waypoint_follower_demo logged_waypoint_follower``
