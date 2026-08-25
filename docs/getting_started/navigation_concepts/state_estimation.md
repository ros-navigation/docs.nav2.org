# State Estimation { #state-estimation }

Within the navigation project, there are 2 major transformations that need to be provided, according to community standards.
The `map` to `odom` transform is provided by a positioning system (localization, mapping, SLAM) and `odom` to `base_link` by an odometry system.

!!! note

    There is **no** requirement on using a LIDAR on your robot to use the navigation system. There is no requirement to use lidar-based collision avoidance, localization, or SLAM. However, we do provide instructions and support tried and true implementations of these things using lidars. You can be equally as successful using a vision or depth based positioning system and using other sensors for collision avoidance. The only requirement is that you follow the standards below with your choice of implementation.

## Standards

[REP 105](https://www.ros.org/reps/rep-0105.html) defines the frames and conventions required for navigation and the larger ROS ecosystem.
These conventions should be followed at all times to make use of the rich positioning, odometry, and SLAM projects available in the community.

In a nutshell, REP-105 says that you must, at minimum, build a TF tree that contains a full `map` -> `odom` -> `base_link` -> `[sensor frames]` for your robot.
TF2 is the time-variant transformation library in ROS 2 we use to represent and obtain time synchronized transformations.
It is the job of the global positioning system (GPS, SLAM, Motion Capture) to, at minimum, provide the `map` -> `odom` transformation.
It is then the role of the odometry system to provide the `odom` -> `base_link` transformation.
The remainder of the transformations relative to `base_link` should be static and defined in your [URDF](http://wiki.ros.org/urdf).

## Global Positioning: Localization and SLAM

It is the job of the global positioning system (GPS, SLAM, Motion Capture) to, at minimum, provide the `map` -> `odom` transformation.
We provide `amcl` which is an Adaptive Monte-Carlo Localization technique based on a particle filter for localization in a static map.
We also provide SLAM Toolbox as the default SLAM algorithm for use to position and generate a static map.

These methods may also produce other output including position topics, maps, or other metadata, but they must provide that transformation to be valid.
Multiple positioning methods can be fused together using robot localization, discussed more below.

## Odometry

It is the role of the odometry system to provide the `odom` -> `base_link` transformation.
Odometry can come from many sources including LIDAR, RADAR, wheel encoders, VIO, and IMUs.
The goal of the odometry is to provide a smooth and continuous local frame based on robot motion.
The global positioning system will update the transformation relative to the global frame to account for the odometric drift.

[Robot Localization](https://github.com/cra-ros-pkg/robot_localization/) is typically used for this fusion.
It will take in `N` sensors of various types and provide a continuous and smooth odometry to TF and to a topic.
A typical mobile robotics setup may have odometry from wheel encoders, IMUs, and vision fused in this manner.

The smooth output can be used then for dead-reckoning for precise motion and updating the position of the robot accurately between global position updates.
