:orphan:

.. _grid_maps:




3. Port Grid Maps to ROS 2 and Environmental Model
==================================================

**Task description** 
Grid Maps was created by ETH Zurich and later transferred to ANYbotics. It is a universal grid map library for mobile robotic mapping that can be used as the basis of environmental models and various forms of grid maps available in ROS 1. This library is one of the top downloaded ROS packages. Your task will be to work with the community and the mentor to port grid_maps metapackage from ROS 1 to ROS 2 and help develop the next generation environment model in ROS 2 to replace costmap_2d.

This will involve porting code from ROS 1 to ROS 2, analyzing uses of the environmental model to define an abstract interface to allow replacement of costmap_2d with grid_map, and building up the basic grid-operations for costmaps. It is not expected to complete the full new model with sensor processing over the course of the summer. If completed early, you may be able to help design a gradient model to complement your implemented costmap model using grid_maps. This will allow robots to select a gradient or a costmap model on startup.

**Project difficulty: High**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**

- Grid Maps ported to ROS 2 and merged into the main ROS 2 branch
- Defined plugin interfaces to replace costmap 2D with grid maps
- Implementing low-level operations on top of grid_maps to replace the base costmap_2d object.
- You will not be expected to reimplement the full sensor processing mechanics of costmap_2d.

**Skills required**

- C++ and Git
- ROS recommended ROS 2, but you can pick it up before starting
- Coordinate transformations and basic geometry
- Recommended: Gazebo simulation and Navigation experience

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Original github issue 1 <https://github.com/ros-planning/navigation2/issues/1278>`_
- `Original github issue 2 <https://github.com/ros-planning/navigation2/issues/1517>`_
- `Navigation2 <https://navigation.ros.org/>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.

