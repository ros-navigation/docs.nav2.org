
.. _dynamic:


1. Navigation Dynamic Obstacle Integration
==========================================

**Task description** 

The Navigation Stack has long provided robust navigation in a wide range of environments. Controllers have been developed to operate effectively in the presence of dynamic obstacles without explicitly modeling the characteristics of dynamic obstacles. However, as the field has progressed and we see more and more robots using ROS deployed in human-filled spaces, more consideration must be taken with respect to dynamic obstacles such as people, carts, animals, and vehicles.

Your task will be to create integrations with existing machine learning tools that create dynamic obstacle information (ComplexYolo, Yolo3D, etc) and tie them into the navigation stack for use. It is not in the scope for you to retrain or otherwise become an expert in 3D machine learning, but some basic knowledge will be helpful. We already have a starting point in the project links below that needs to be driven to completion. This includes completing the on-going work to integrate yolact edge into this work to replace detectron2 and benchmark these capabilities on GPUs to verify sufficient run-time performance, as well as other tangental feature development.

This task will involve identifying a few techniques that produce position and velocity information about dynamic obstacles that can run on a mobile robot (using high-power Intel CPU, Nvidia Jetson SoC, external GPUs, etc) and get them running with ROS and Navigation. Next, you will help create a new costmap layer to use this information to mark the dynamic obstacle in the costmap to ensure a robot does not collide with a future trajectory of an obstacle.

If time permits, you may also work to also integrate this dynamic information into a path planner and/or controller to help in direct motion consideration. This will likely be in collaboration with another community member.

**Project difficulty: High**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**
- Integrations of 1 method of dynamic obstacle detection in ROS 2, using machine learning
- Test these capabilities on real data or a live robot to demonstrate functionality
- 85% test coverage or higher

**Skills required**

- C++, Python, ROS
- Mobile robot navigation experience
- Geometry and statistics
- Recommended: Gazebo simulation, machine learning, ROS navigation

**List of relevant open source software repositories and refs** 

- `Starting project <https://github.com/ros-planning/navigation2_dynamic/>`_
- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Github ticket <https://github.com/ros-planning/navigation2/issues/1617>`_
- `Navigation2 <https://navigation.ros.org/>`_
- `Some related works <https://alyssapierson.files.wordpress.com/2018/05/pierson2018.pdf>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.
