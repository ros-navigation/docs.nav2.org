:orphan:

.. _create_plugins:



2. Create New Planner and Controller Plugins
============================================

**Task description**

The ROS 2 Navigation Stack has a number of plugin interfaces to help users create or select specific plugins for planning, control, and behaviors for their applications. Two specific areas that the Nav2 stack could use more algorithm plugins for is for path planning (referred to as a planner plugin) and local trajectory generation (referred to as controller plugins). A simple tutorial for creating a `planner plugin can be found here. <https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html>`_ Currently, we have one planner, NavFn which implements an A* and Dijkstra's planner. It also has two controllers, DWB and TEB which implement a DWA and timed elastic-band optimization techniques. There is also a Hybrid-A* and OMPL planner in development.

Your task will be to create a high-quality implementation of one of the following algorithms for the Nav2 plugin interfaces. Alternative algorithms may also be considered upon approval, please ask @steve in the application phase. Please select only one to discuss.

- Planner Plugin Options: D* or variant, Vornoi planner, Navigation graph route planner, State Lattice planner, kinodynamic planner, and any planning method given a set of dynamic and static obstacles.
- Controller Plugin Options: CiLQR, iLQR, MPC, Splines, path following or dynamic obstacle following controllers.
- Additional options: helping in completing the OMPL or Hybrid-A* planner.

**Project difficulty: High**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**

- A functional planner or controller plugin for the Nav2 stack
- Plugin should be optimized for run-time performance with 50% or greater test coverage

**Skills required**

- C++
- Path planning or motion planning
- Algorithm optimization
- ROS / Pluginlib
- Recommended: Gazebo simulation and Navigation experience

**List of relevant open source software repositories and refs**

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Github issue page <https://github.com/ros-planning/navigation2/issues/1710>`_
- `Nav2 <https://navigation.ros.org/>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.

