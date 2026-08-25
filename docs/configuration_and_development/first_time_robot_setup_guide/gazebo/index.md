# Setup Guide for Gazebo { #setup-guide-for-gazebo }

This guide covers setting up Nav2 with the modern Gazebo simulator (Gazebo Harmonic or newer), used with ROS 2 Jazzy or newer.

Follow these tutorials in order to set up your robot for Nav2:

<div class="grid cards bottom-align" markdown>

- :material-axis-arrow: **Setting Up Transformations**

    ---
    Configure TF2 transforms between robot frames.

    [:octicons-arrow-right-24: Go][setting-up-transformations]

- :material-cube-outline: **Setting Up The URDF**

    ---
    Create the robot description your robot.

    [:octicons-arrow-right-24: Go][setting-up-the-urdf]

- :material-cube: **Setting Up The SDF - Gazebo**

    ---
    Create the simulation model for Gazebo.

    [:octicons-arrow-right-24: Go][setting-up-the-sdf-gazebo]

- :material-speedometer: **Setting Up Odometry - Gazebo**

    ---
    Configure wheel encoders and odometry publishers.

    [:octicons-arrow-right-24: Go][setting-up-odometry-gazebo]

- :material-chart-bell-curve: **Smoothing Odometry using Robot Localization**

    ---
    Fuse IMU and wheel odometry with an EKF for smoother estimates.

    [:octicons-arrow-right-24: Go][smoothing-odometry-using-robot-localization]

- :material-radar: **Setting Up Sensors - Gazebo**

    ---
    Configure lidar, depth cameras, and other perception sensors.

    [:octicons-arrow-right-24: Go][setting-up-sensors-gazebo]

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
