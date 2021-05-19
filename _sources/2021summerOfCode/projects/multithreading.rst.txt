
.. _multhrreading:

4. Navigation MultiThreading
============================

**Task description** 

The aim of this project is the significantly improve the run-time performance of Nav2 making sure to leverage the full capabilities of multi-processor core CPUs. We seek to identify areas in the Nav2 stack that could leverage multi-threading or parallel processing to speed up computations and improve overall user performance on a broad range of compute platforms.

Some examples include:
- AMCL particle cloud updates
- Costmap layer updates
- Costmap sensor data population
- Controller critic evalulation
- Collision checking
- Voxel grid ray casting
- and more.

We are seeking a student interested in learning about multi-threading and parallel processing, ideally with some exposure to these concepts and libraries already, to analyze potential areas for parallel computing. Then, select the top candidates and implement them with parallel processing and benchmark the improvements to the Nav2 stack they provide.

This will be an excellent chance to apply (or obtain) C++ parallel computing skills while also learning a great deal about how to build mobile robot navigation systems -- both very valuable skillsets. 

**Project difficulty: Medium**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**
- Analysis of potential areas in the Nav2 stack that can benefit from parallel processing
- Integrations of 3-5 areas in the Nav2 stack that non-trivially improves run-time performance of the stack using multithreading
- Benchmark performance improvements

**Skills required**

- C++, ROS
- Mobile robot navigation experience
- Working knowledge (or ability to quickly obtain) on one or more of: TBB, OpenMP, OpenCL, Cuda, and similar
- Recommended: Gazebo simulation, ROS navigation

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Github ticket <https://github.com/ros-planning/navigation2/issues/2042>`_
- `Navigation2 <https://navigation.ros.org/>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.
