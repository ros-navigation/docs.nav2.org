
.. _dynamic:


7. Navigation Dynamic Obstacle Integration
==========================================

**Task description** 

The Navigation Stack has long provided robust navigation in a wide range of environments. Controllers have been developed to operate effectively in the presence of dynamic obstacles without explicitly modeling the characteristics of dynamic obstacles. However, as the field has progressed and we see more and more robots using ROS deployed in human-filled spaces, more consideration must be taken with respect to dynamic obstacles such as people, carts, animals, and vehicles.

Your task will be to create integrations with existing machine learning tools that create dynamic obstacle information (ComplexYolo, Yolo3D, etc) and tie them into the navigation stack for use. It is not in the scope for you to retrain or otherwise become an expert in 3D machine learning, but some basic knowledge will be helpful.

This task will involve identifying a few techniques that produce position and velocity information about dynamic obstacles that can run on a mobile robot (using high-power Intel CPU, Nvidia Jetson SoC, external GPUs, etc) and get them running with ROS and Navigation. Next, you will help create a new costmap layer to use this information to mark the dynamic obstacle in the costmap to ensure a robot does not collide with a future trajectory of an obstacle.

If time permits, you may also work to also integrate this dynamic information into a path planner and/or controller to help in direct motion consideration. This will likely be in collaboration with another community member.

**Project difficulty: High**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**
- Integrations of 2-5 methods of dynamic obstacle detection in ROS 2
- Creation of a costmap_2d layer to use this information to mark future trajectories into a costmap
- Test these capabilities on real data or a live robot to demonstrate functionality
- 65% test coverage or higher

**Skills required**

- C++, Python, ROS
- Mobile robot navigation experience
- Geometry and statistics
- Recommended: Gazebo simulation, machine learning, ROS navigation

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `OGithub ticket <https://github.com/ros-planning/navigation2/issues/1617>`_
- `Navigation2 <https://navigation.ros.org/>`_
- `Some related works <https://alyssapierson.files.wordpress.com/2018/05/pierson2018.pdf>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.
