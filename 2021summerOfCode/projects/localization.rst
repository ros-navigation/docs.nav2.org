:orphan:

.. _localization:


6. 2D/3D Localization Improvements
==================================

**Task description** 

The Navigation2 stack uses AMCL as its primary localization engine. Over the last 10 years, essentially no updates to AMCL has been made. This is due to the code base for this implementation of an Adaptive Monte Carlo Localizer is written in embedded C, not well structured, and very sensitive to changes. A-MCL implementations have been a hallmark of localization for over a decade but this particular implementation should be deprecated. 

Your target involves designing and creating a new localization engine for the Nav2 stack. The requirements of this are:
- Support 2D laser scanners
- Support 3D laser scanners, where 2D case could potentially be a simplified case
- Accurately track the localization of a robot in a given occupancy grid

The reason that specific method is left open-ended is to allow for creativity, novelty, or reimplementation of a what you feel is best. We have, however, analyzed other MCL variants as being good options. This may include reimplementing an A-MCL that is designed to be modified with modular components and support sampling from a 3D lidar. Another option is a NDT-MCL using NDT 2D/3D scan matching. Other options may be proposed and discussed with mentors during the application phase. The task involves 3D as well since there is no standard 3D localizer in ROS 2 yet and more and more robust 3D SLAM libraries have emerged over the last 2 years. 

An optional but recommended feature of this work would be to also accept the inputs from multiple laser scanners. However it is not strictly required.

**Project difficulty: High**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**
- 2D and 3D localization system based on laser scanners
- 65% or higher test coverage
- Designed with modular components that can be reliably modified over time


**Skills required**

- C/C++
- Localization, particle filter, or SLAM techniques
- Ability to read and implement academic works
- Recommended: Gazebo simulation and Navigation experience

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Original Github Issue <https://github.com/ros-planning/navigation2/issues/1391>`_
- `Navigation2 <https://navigation.ros.org/>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.



