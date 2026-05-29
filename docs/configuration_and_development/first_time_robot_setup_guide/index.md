# First-Time Robot Setup Guide { #first-time-robot-setup-guide }

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

Choose the appropriate guide based on your ROS 2 distribution:

<div style="display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin: 20px 0;" markdown>
<div markdown>

- [Setup Guide for Gazebo][setup-guide-for-gazebo]
    - [Setting Up Transformations][setting-up-transformations]
    - [Setting Up The URDF][setting-up-the-urdf]
    - [Setting Up The SDF - Gazebo][setting-up-the-sdf-gazebo]
    - [Setting Up Odometry - Gazebo][setting-up-odometry-gazebo]
    - [Smoothing Odometry using Robot Localization][smoothing-odometry-using-robot-localization]
    - [Setting Up Sensors - Gazebo][setting-up-sensors-gazebo]
    - [Mapping and Localization][mapping-and-localization]
    - [Setting Up the Robot's Footprint][setting-up-the-robots-footprint]
    - [Setting Up Navigation Plugins][setting-up-navigation-plugins]

</div>
<div markdown>

- [Setup Guide for Gazebo Classic][setup-guide-for-gazebo-classic]
    - [Setting Up Transformations][setting-up-transformations]
    - [Setting Up The URDF][setting-up-the-urdf]
    - [Setting Up Odometry - Gazebo Classic][setting-up-odometry-gazebo-classic]
    - [Smoothing Odometry using Robot Localization][smoothing-odometry-using-robot-localization]
    - [Setting Up Sensors - Gazebo Classic][setting-up-sensors-gazebo-classic]
    - [Mapping and Localization][mapping-and-localization]
    - [Setting Up the Robot's Footprint][setting-up-the-robots-footprint]
    - [Setting Up Navigation Plugins][setting-up-navigation-plugins]

</div>
</div>
