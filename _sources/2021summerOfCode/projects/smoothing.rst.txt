
.. _smoothing:

4. Path Smoothing
=================

**Task description** 

This project is to create a generic path smoothing node or library to help in smoothing out paths generated from search-based planners used in Nav2 (Dijkstra, A*, NavFn, Hybrid-A*, State Lattice, etc) to make them more drivable at higher speeds and remove any non-optimal artifacts from the searching process due to imperfect heuristic functions.

Common ways to smooth paths include:
- Non-linear optimization
- Gradient Descent (related)
- Splines
- Curve fitting
- etc

Your task will be to create a path smoother that will take in a given path and smooth it to be more drivable and return that path to the requester.

**Project difficulty: Medium**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**
- Can run in real-time (< 100ms) on average
- Can smooth paths of arbitrary lengths and from arbitrary path planning algorithms used in Nav2
- Returned paths with tangent orientation vectors included (see ticket 2)
- May be in Python or C++
- If time allots, and in Python, convert to C++

**Skills required**

- C++, Python, ROS
- Mobile robot navigation experience
- Geometry, splines, and/or optimization
- Recommended: Gazebo simulation, ROS navigation

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Github ticket <https://github.com/ros-planning/navigation2/issues/2230>`_
- `Github ticket2 <https://github.com/ros-planning/navigation2/issues/2162>`_
- `Navigation2 <https://navigation.ros.org/>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.
