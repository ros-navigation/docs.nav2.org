# First-Time Robot Setup Guide { #first-time-robot-setup-guide }

This section is a collection of guides that aims to provide readers a good resource for setting up Nav2. The objectives for this section are as follows:

- Help new users with setting up Nav2 with a new robot
- Help people with custom built robots to properly set up their robots to be used in ROS/Nav2
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

    This guide covers setting up Nav2 with the modern Gazebo simulator (Gazebo Harmonic or newer), used with ROS 2 Jazzy or newer.

Follow these tutorials in order to set up your robot for Nav2:

<div class="grid cards" markdown>

- :material-axis-arrow: **Setting Up Transformations**

    ---
    Configure TF2 transforms between robot frames.

    [:octicons-arrow-right-24: Go][setting-up-transformations]

- :material-cube-outline: **Setting Up The URDF**

    ---
    Create the robot description your robot.

    [:octicons-arrow-right-24: Go][setting-up-the-urdf]

- :material-cube: **Setting Up The SDF**

    ---
    Create the simulation model for Gazebo.

    [:octicons-arrow-right-24: Go][setting-up-the-sdf]

- :material-speedometer: **Setting Up Odometry**

    ---
    Configure wheel encoders and odometry publishers.

    [:octicons-arrow-right-24: Go][setting-up-odometry]

- :material-chart-bell-curve: **Smoothing Odometry using Robot Localization**

    ---
    Fuse IMU and wheel odometry with an EKF for smoother estimates.

    [:octicons-arrow-right-24: Go][smoothing-odometry-using-robot-localization]

- :material-radar: **Setting Up Sensors**

    ---
    Configure lidar, depth cameras, and other perception sensors.

    [:octicons-arrow-right-24: Go][setting-up-sensors]

- :material-map-search: **Mapping and Localization**

    ---
    Create maps with SLAM and localize with AMCL.

    [:octicons-arrow-right-24: Go][mapping-and-localization]

- :material-shape-outline: **Setting Up the Robot's Footprint**

    ---
    Define the robot's physical footprint for collision checking.

    [:octicons-arrow-right-24: Go][setting-up-the-robots-footprint]

- :material-puzzle-outline: **Setting Up Navigation Plugins**

    ---
    Select and configure planner, controller, and behavior plugins.

    [:octicons-arrow-right-24: Go][setting-up-navigation-plugins]

</div>

!!! note

    These tutorials are not meant to be full tuning and configuration guides since they only aim to help you get your robot up and running with a basic configuration. For more detailed discussions and guides on how to customize and tune Nav2 for your robot, head on to the [Configuration Guide][configuration-guide] section.
