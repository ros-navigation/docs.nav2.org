
.. _semantics:

6. Semantic Integration
=======================

**Task description**

This project is to create a semantics library and integrate it into the Nav2 system. The major aim is to enable a generic semantic representation format that navigation (and potentially other ROS projects) may use for representing their environment, objects within it, or points of interest.

After creating the generic representation, your project will be to create demonstrations within Nav2 using this capability including a route planning server to replace the planner server in situations where you have a pre-defined set of potential locales (non-free space planning) and another demonstration of your choice.

**Project difficulty: Hard**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**
- Generic semantics standard added to Nav2 documentation
- Generic semantics ROS 2 library that implements the standard and makes it easy for applications to get, retreive, or analyze semantic data for custom purposes
- A route server to enable navigation-graph and/or route following capabilities
- 1 more demonstration using the semantics library of your choice (could be costmap layer with different rules in different rooms or with different objects, a multi-story building demo using semantic info to allow a robot to plan and execute multi-story trajectories, etc)

**Skills required**

- C++, Python, ROS 2
- Mobile robot navigation or manipulation experience
- Perception, semantic information motivation, or similar.
- Recommended: Gazebo simulation, ROS navigation

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Github ticket <https://github.com/ros-planning/navigation2/issues/1595>`_
- `Github ticket2 <https://github.com/ros-planning/navigation2/issues/2229>`_
- `Navigation2 <https://navigation.ros.org/>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.
