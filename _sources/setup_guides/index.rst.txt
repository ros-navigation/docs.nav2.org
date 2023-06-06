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

- Introduce TF2 and setup your robot URDF
- Setup sensor sources for robot odometry
- Setup sensor sources for perception
- Configure round or arbitrary shaped footprints for your robot
- Select and set up planner and controller navigation plugins for your robot's navigation tasks  
- Lifecycle node management for easy bringup of other related sensors or nodes

.. note:: These tutorials are not meant to be full tuning and configuration guides since they only aim to help you get your robot up and running with a basic configuration. For more detailed discussions and guides on how to customize and tune Nav2 for your robot, head on to the :ref:`configuration` section.

**Table of Contents:**

.. toctree::
   :maxdepth: 1

   transformation/setup_transforms.rst
   urdf/setup_urdf.rst
   odom/setup_odom.rst
   sensors/setup_sensors.rst
   footprint/setup_footprint.rst
   algorithm/select_algorithm.rst
