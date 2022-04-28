
.. _safety_node:

5. Navigation Safety Node
=========================

**Task description** 

The aim of this project is to create a safety watchdog node to ensure the robot is acting properly and not about to collide with an obstacle. Typical safety-rated lidars will contain "safety zones" whereas if any sensor points are located in a box around the lidar, then the lidar will send a signal to the robot to stop due to a potential collision. However, less and less people are using safety-rated lidars as consumer available lidars are dropping in cost and 3D lidars are seeing more use in mobile robotics.

Your project will be to re-create this logic at the Navigation level. While this wouldn't be "safety certified", this is a significant functional improvement on safety that could potentially safe real people from real injuries in the real-world. The project will be to create a node that sits below the navigation stack but above the robot controller to do the following:

- Take in the current command velocity from navigation and the most recent laser or RGBD scan
- Projecting the velocity forward in time ``N`` seconds, check if that velocity will result in a collision with any sensor measurements
- If not, allow the velocity command through to the base
- If it does collide, scale back the velocity such that the robot will always be at minimum ``N`` seconds from a collision
- Optionally if a flag is set, if ``M`` or more points are in defined bounding boxes around the robot, send only ``0`` commands to enact an emergency stop. 

This will be an excellent chance to make mobile robots and Nav2 users significantly safer and run at higher speeds in their production or research environments. It goes a long way for functional safety for those not using safety-rated lidars which contain similar features.

**Project difficulty: Medium**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**
- Creation of a ROS 2 node that will prevent robot collision based on lidar and/or RGBD data
- Node should be able to run at lidar data speed (40hz+) and adjust velocity commands accordingly
- If time allots, work on tuning / adding new critics to the DWB local planner to improve safety of its performance for users out of the box

**Skills required**

- C++, ROS
- Mobile robot navigation experience
- Geometry and statistics
- Recommended: Gazebo simulation, ROS navigation

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Github ticket <https://github.com/ros-planning/navigation2/issues/1899>`_
- `Navigation2 <https://navigation.ros.org/>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.
