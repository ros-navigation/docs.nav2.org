.. _navigation2-with-gps:

Navigating with a GPS Based Localization System
**************************************

- `Overview`_
- `Requirements`_
- `GPS Localization Overview`_
- `Tutorial Steps`_

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700" height="450" src="https://www.youtube.com/embed/ZeCds7Sv-5Q?autoplay=1" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
      </div>
    </h1>

Overview
========

This tutorial shows how to set up a localization system using a GPS sensor as source of global positioning and how to use Nav2 to follow GPS waypoints.

Requirements
============

It is assumed ROS2 and Nav2 dependent packages are installed or built locally. Additionally you will have to install robot_localization and mapviz: 

   .. code-block:: bash

      sudo apt install ros-<ros2-distro>-robot-localization
      sudo apt install ros-<ros2-distro>-mapviz
      sudo apt install ros-<ros2-distro>-mapviz-plugins
      sudo apt install ros-<ros2-distro>-tile-map

GPS Localization Overview
============

GPS (Global Positioning System) or more broadly GNSS (Global Navigation Satellite System) is a technology that relies on satellites to provide receivers with an estimate of where they are located on the earth. These satellites are located at altitudes around 20.000km and use radio frequency to continuously broadcast time signals, these are picked up by receivers when satellites are along their line of sight, they use trilateration to estimate their latitude, longitude and altitude.

GPS devices calculate their position using the `WGS84 standard <https://en.wikipedia.org/wiki/World_Geodetic_System>`_, which defines a cartesian system the earth's origin on its center of mass, the `z` axis pointing north and the `x` axis pointing to the first meridian as the image below shows.

.. image:: images/Gps_Navigation/WGS_84_reference_frame.svg
    :width: 562px
    :align: center
    :alt: WGS84 reference frame

However, this reference system is impractical for describing the motion of objects in or close to the earth's surface: Imagine your robot is located on a soccer field and you want it to move from one end to the other, your navigation task would look something like "go from X=4789.413 km, Y=177.511 km z=4194.292 km to X=4789.475 km, Y=177.553 km z=4194.22 km". It would make much more sense to create a local reference system where you could tell your robot "go 100 meters forward", right?

To cope with this, geodesy allows for the definition of  `Local tangent planes <https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates>`_. According to `REP 103 <https://www.ros.org/reps/rep-0103.html>`_ the coordinate systems of these planes should follow the ENU convention, which means the +x axis should face **E**ast, +y axis should face **N**orth and the +z axis should point **U**p, as the image below shows. In the context of this tutorial the GPS coordinates of the origin of the local cartesian plane will be called **datum** as it is in `robot_localization <http://docs.ros.org/en/noetic/api/robot_localization/html/index.html>`_

.. image:: images/Gps_Navigation/ECEF_ENU_Longitude_Latitude_relationships.svg
    :width: 520px
    :align: center
    :alt: Local tangent plane

Tutorial Steps
==============

0- Setup Your Enviroment Variables
----------------------------------

Run the following commands first whenever you open a new terminal during this tutorial.

- ``source /opt/ros/<ros2-distro>/setup.bash``
- ``export TURTLEBOT3_MODEL=waffle``

1- Launch Turtlebot 3
---------------------

You will need to launch your robot's interface,

  ``ros2 launch turtlebot3_bringup robot.launch.py  use_sim_time:=False``

2- Launch Nav2
--------------

You need to have a map of the environment where you want to Navigate Turtlebot 3, or create one live with SLAM.

In case you are interested, there is a use case tutorial which shows how to use Nav2 with SLAM.
:ref:`navigation2-with-slam`.

Required files:

   - ``your-map.map``
   - ``your-map.yaml``

``<your_map>.yaml`` is the configuration file for the map we want to provide Nav2.
In this case, it has the map resolution value, threshold values for obstacles and free spaces, and a map file location.
You need to make sure these values are correct.
More information about the map.yaml can be found `here <http://wiki.ros.org/map_server>`_.

Launch Nav2. If you set autostart:=False, you need to click on the start button in RViz to initialize the nodes.
Make sure `use_sim time` is set to **False**, because we want to use the system time instead of the time simulation time from Gazebo.

``ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=False map:=/path/to/your-map.yaml``

Note: Don't forget to change **/path/to/your-map.yaml** to the actual path to the your-map.yaml file.

3-  Launch RVIZ
---------------

Launch RVIZ with a pre-defined configuration file.

  ``ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz``

Now, you should see a shadow of Turtlebot 3 robot model in the center of the plot in Rviz.
Click on the Start button (Bottom Left) if you set the auto_start parameter to false.
Then, the map should appear in RViz.

.. image:: images/Navigation2_on_real_Turtlebot3/rviz_after_launch_view.png
    :width: 48%
.. image:: images/Navigation2_on_real_Turtlebot3/rviz_slam_map_view.png
    :width: 45%

4- Initialize the Location of Turtlebot 3
-----------------------------------------

First, find where the robot is on the map. Check where your robot is in the room.

Set the pose of the robot in RViz.
Click on the 2D Pose Estimate button and point the location of the robot on the map. 
The direction of the green arrow is the orientation of Turtlebot.

.. image:: images/Navigation2_on_real_Turtlebot3/rviz_set_initial_pose.png
    :width: 700px
    :align: center
    :alt: Set initial pose in RViz

Now, the 3D model of Turtlebot should move to that location. 
A small error in the estimated location is tolerable.

5-  Send a Goal Pose
--------------------

Pick a target location for Turtlebot on the map. 
You can send Turtlebot 3 a goal position and a goal orientation by using the **Nav2 Goal** or the **GoalTool** buttons.

Note: Nav2 Goal button uses a ROS 2 Action to send the goal and the GoalTool publishes the goal to a topic.

.. image:: images/Navigation2_on_real_Turtlebot3/rviz_send_goal.png
    :width: 700px
    :align: center
    :alt: Send goal pose in RViz

Once you define the target pose,  Nav2 will find a global path and start navigating the robot on the map.

.. image:: images/Navigation2_on_real_Turtlebot3/rviz_robot_navigating.png
    :width: 700px
    :align: center
    :alt: Robot navigating in RViz

Now, you can see that Turtlebot 3 moves towards the goal position in the room. See the video below.
