.. _navigation2-on-real-turtlebot3:

Navigating with a Physical Turtlebot 3
**************************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700" height="450" src="https://www.youtube.com/embed/ZeCds7Sv-5Q?autoplay=1" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
      </div>
    </h1>

Overview
========

This tutorial shows how to control and navigate Turtlebot 3 using the ROS 2 Nav2 on a physical Turtlebot 3 robot.
Before completing this tutorials, completing :ref:`getting_started` is highly recommended especially if you are new to ROS and Nav2.

This tutorial may take about 1 hour to complete. 
It depends on your experience with ROS, robots, and what computer system you have.

Requirements
============

You must install Nav2, Turtlebot3.
If you don't have them installed, please follow :ref:`getting_started`.

Tutorial Steps
==============

0- Setup Your Environment Variables
-----------------------------------

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
