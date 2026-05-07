.. _setup_guides_gazebo:

Setup Guide for Gazebo
#######################

This guide covers setting up Navigation2 with the modern Gazebo simulator (Gazebo Harmonic or newer), used with ROS 2 Jazzy or newer.

Follow these tutorials in order to set up your robot for Nav2:

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

.. note:: These tutorials are not meant to be full tuning and configuration guides since they only aim to help you get your robot up and running with a basic configuration. For more detailed discussions and guides on how to customize and tune Nav2 for your robot, head on to the :ref:`configuration` section.
