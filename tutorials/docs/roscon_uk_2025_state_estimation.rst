.. _roscon-uk-2025-state-estimation:

ROSCon UK 2025: State Estimation for Mobile Robotics Workshop
*************************************************************

- `Overview`_
- `Workshop Content`_
- `Learning Outcomes`_
- `Getting Started`_

Overview
========

This tutorial provides comprehensive hands-on experience with state estimation techniques for mobile robotics, presented as a workshop at ROSCon UK 2025.
State estimation is a fundamental component of any autonomous mobile robot system, enabling robots to maintain accurate knowledge of their position, orientation, and velocity in the world.
This workshop covers two complementary approaches to state estimation that are widely used in production robotics systems.

The workshop materials explore both classical filtering approaches and modern optimization-based methods for sensor fusion.
Whether you're building a warehouse robot, an outdoor autonomous vehicle, or any other mobile platform, understanding these state estimation techniques is crucial for achieving reliable navigation and localization performance.
The hands-on exercises use real ROS 2 packages that are production-tested and widely deployed in commercial robotics applications.

The complete workshop materials, including setup instructions, exercises, and solutions, are available in the official repository:

`ROSCon UK 2025 State Estimation Workshop Repository <https://github.com/locusrobotics/roscon-uk-2025-se-workshop>`_

Workshop Content
================

The workshop covers two powerful state estimation frameworks:

**Robot Localization - Kalman Filter-Based Sensor Fusion**

Robot Localization is a ROS package that provides Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF) implementations for fusing data from multiple sensors.
This approach is particularly well-suited for real-time state estimation where computational efficiency is critical.
The workshop demonstrates how to configure Robot Localization to fuse IMU data, wheel odometry, GPS, and other sensors to produce smooth, accurate state estimates.
Topics include:

- Understanding the Extended Kalman Filter and Unscented Kalman Filter algorithms
- Configuring sensor inputs and fusion parameters
- Tuning process and measurement noise covariances
- Setting up coordinate frame transforms for proper sensor fusion
- Handling different sensor update rates and characteristics
- Debugging common state estimation issues

**Fuse - Factor Graph Optimization**

Fuse is a general architecture for performing sensor fusion using factor graphs.
Unlike traditional filtering approaches, factor graph optimization considers a history of measurements to jointly optimize the robot's trajectory.
This batch optimization approach can produce more accurate results by reasoning about past measurements and future constraints simultaneously.
The workshop covers:

- Introduction to factor graph concepts and optimization
- Building factor graphs for mobile robot state estimation
- Integrating various sensor modalities into the factor graph framework
- Understanding the trade-offs between filtering and optimization approaches
- Fixed-lag smoothing for real-time performance
- Loop closure detection and handling with factor graphs

Both approaches have their strengths.
The workshop provides practical experience with both methods, enabling you to choose the right approach for your specific robotics application.

Check out the full workshop materials in the repository: https://github.com/locusrobotics/roscon-uk-2025-se-workshop
