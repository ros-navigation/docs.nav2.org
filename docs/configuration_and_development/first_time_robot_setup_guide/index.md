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

!!! note

    This guide covers setting up Navigation2 with the modern Gazebo simulator (Gazebo Harmonic or newer), used with ROS 2 Jazzy or newer.

Follow these tutorials in order to set up your robot for Nav2:

<div class="grid" markdown>

[Setting Up Transformations][setting-up-transformations]{ .md-button .md-button--primary }
[Setting Up The URDF][setting-up-the-urdf]{ .md-button .md-button--primary }
[Setting Up The SDF][setting-up-the-sdf]{ .md-button .md-button--primary }
[Setting Up Odometry][setting-up-odometry]{ .md-button .md-button--primary }
[Smoothing Odometry using Robot Localization][smoothing-odometry-using-robot-localization]{ .md-button .md-button--primary }
[Setting Up Sensors][setting-up-sensors]{ .md-button .md-button--primary }
[Mapping and Localization][mapping-and-localization]{ .md-button .md-button--primary }
[Setting Up the Robot's Footprint][setting-up-the-robots-footprint]{ .md-button .md-button--primary }
[Setting Up Navigation Plugins][setting-up-navigation-plugins]{ .md-button .md-button--primary }

</div>

!!! note

    These tutorials are not meant to be full tuning and configuration guides since they only aim to help you get your robot up and running with a basic configuration. For more detailed discussions and guides on how to customize and tune Nav2 for your robot, head on to the [Configuration Guide][configuration-guide] section.
