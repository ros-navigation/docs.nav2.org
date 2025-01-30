.. _setup_guides:

First-Time Robot Setup Guide
############################

This section is a collection of guides that aims to provide readers a good resource for setting up Nav2. The objectives for this section are as follows:

- Help new users with setting up Navigation2 with a new robot
- Help people with custom built robots to properly set up their robots to be used in ROS/Navigation2
- Act as a checklist, template or boilerplate reference for more experienced readers
- Provide examples which can be run on simulators/tools like Gazebo or RViz to guide readers on the Nav2 setup process even without a physical robot.
- Broad strokes, tips, and tricks for configuring certain packages and integrating different components of the robot platform (sensors, odometry, etc.)

To guide you through the first-time setup of your robot, we will be tackling the following topics:

- Introduce TF2 and setup your robot URDF & SDF

- Setup sensor sources for robot odometry
- Setup sensor sources for perception
- Configure round or arbitrary shaped footprints for your robot
- Select and set up planner and controller navigation plugins for your robot's navigation tasks  
- Lifecycle node management for easy bringup of other related sensors or nodes

There are tutorials below for the Gazebo Classic simulator and the Gazebo simulator. Gazebo Classic is used with ROS 2 Humble and previous ROS distributions, while Gazebo (Gazebo Harmonic or newer) is used with ROS 2 Jazzy or newer.


Table of Contents
=================

.. raw:: html  

    <div style="display: flex; gap: 20px;">  

    <div style="flex: 1; padding: 10px; border-right: 1px solid #ccc;">  

    <h3 style="text-align: center;">Gazebo</h3>

.. toctree::
   :maxdepth: 1

   transformation/setup_transforms.rst
   urdf/setup_urdf.rst
   sdf/setup_sdf.rst
   odom/setup_odom_gz.rst
   odom/setup_robot_localization.rst
   sensors/setup_sensors_gz.rst
   sensors/mapping_localization.rst
   footprint/setup_footprint.rst
   algorithm/select_algorithm.rst

.. raw:: html

    </div>

    <div style="flex: 1; padding: 10px;">

    <h3 style="text-align: center;">Gazebo Classic</h3>

.. toctree::
   :maxdepth: 1

   transformation/setup_transforms.rst
   urdf/setup_urdf.rst
   odom/setup_odom_gz_classic.rst
   odom/setup_robot_localization.rst
   sensors/setup_sensors_gz_classic.rst
   sensors/mapping_localization.rst
   footprint/setup_footprint.rst
   algorithm/select_algorithm.rst

.. raw:: html

    </div>
    </div>

.. note:: These tutorials are not meant to be full tuning and configuration guides since they only aim to help you get your robot up and running with a basic configuration. For more detailed discussions and guides on how to customize and tune Nav2 for your robot, head on to the :ref:`configuration` section.
