.. _getting_started:

Getting Started
###############

This document will take you through the process of installing the |PN| binaries
and navigating a simulated Turtlebot 3 in the Gazebo simulator.

.. note::

  See the :ref:`build-instructions` for other situations such as building from source or
  working with other types of robots.

.. warning::

  This is a simplified version of the Turtlebot 3 instructions. We highly
  recommend you follow the `official Turtlebot 3 manual`_ if you intend to
  continue working with this robot beyond the minimal example provided here.

Installation
************

Jazzy introduced the new Gazebo modern simulator, replacing Gazebo Classic.
Thus, for Jazzy and newer, the installation packages and instructions are slightly different to pull in the appropriate packages.

1. Install the `ROS 2 binary packages`_ as described in the official docs
2. Install the |PN| packages using your operating system's package manager:

   .. code-block:: bash

      sudo apt install ros-<ros2-distro>-navigation2
      sudo apt install ros-<ros2-distro>-nav2-bringup

3. Install the demo robot (Turtlebot) for gazebo:

For **Jazzy and newer**, install the Turtlebot 3 & 4 packages for Gazebo Modern. It should be automatically installed with ``nav2_bringup``:

   .. code-block:: bash

      sudo apt install ros-<ros2-distro>-nav2-minimal-tb*


For **Iron and older**, install Turtlebot 3 packages for gazebo classic:

   .. code-block:: bash

      sudo apt install ros-<ros2-distro>-turtlebot3-gazebo

Running the Example
*******************

1. Start a terminal in your GUI
2. Set key environment variables, some of which are only required for Iron and older:

   .. code-block:: bash

      source /opt/ros/<ros2-distro>/setup.bash
      export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic
      export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models # Iron and older only with Gazebo Classic

3. In the same terminal, run:

   .. code-block:: bash

      ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
   
   .. note::
      
      ``headless`` defaults to true; if not set to false, gzclient (the 3d view) is not started.

   This launch file will launch Nav2 with the AMCL localizer in the
   simulation world.
   It will also launch the robot state publisher to provide transforms,
   a Gazebo instance with the Turtlebot3 URDF, and RVIZ.

   If everything has started correctly, you will see the RViz and Gazebo GUIs like
   this (this is Gazebo Classic, but what you see with modern Gazebo is virtually identical):

   .. image:: /images/rviz/rviz-not-started.png
      :width: 45%
   .. image:: /images/gazebo/gazebo_turtlebot1.png
      :width: 46%

4. If not autostarting, click the "Startup" button in the bottom left corner of RViz.
   This will cause |PN| to change to the Active state. It should
   change appearance to show the map.

   .. image:: /images/rviz/rviz_initial.png
      :width: 700px
      :align: center
      :alt: Initial appearance of RViz transitioning to the Active state

Navigating
**********

After starting, the robot initially has no idea where it is. By default,
|PN| waits for you to give it an approximate starting position. Take a look
at where the robot is in the Gazebo world, and find that spot on the map. Set
the initial pose by clicking the "2D Pose Estimate" button in RViz, and then
down clicking on the map in that location. You set the orientation by dragging
forward from the down click.

If you are using the defaults so far, the robot should look roughly like this.

   .. image:: /images/rviz/rviz-set-initial-pose.png
      :width: 700px
      :align: center
      :alt: Approximate starting location of Turtlebot

If you don't get the location exactly right, that's fine. |PN| will refine
the position as it navigates. You can also, click the "2D Pose
Estimate" button and try again, if you prefer.

Once you've set the initial pose, the transform tree will be complete and
|PN| will be fully active and ready to go. You should see the robot and particle
cloud now.

   .. image:: /images/rviz/navstack-ready.png
      :width: 700px
      :align: center
      :alt: |PN| is ready. Transforms and Costmap show in RViz.

Next, click the "Navigaton2 Goal" button and choose a destination.
This will call the BT navigator to go to that goal through an action server.
You can pause (cancel) or reset the action through the Nav2 rviz plugin shown.

   .. image:: /images/rviz/navigate-to-pose.png
      :width: 700px
      :align: center
      :alt: Setting the goal pose in RViz.

Now watch the robot go!

.. image:: images/navigation_with_recovery_behaviours.gif
    :width: 700px
    :alt: Navigation2 with Turtlebot 3 Demo
    :align: center
