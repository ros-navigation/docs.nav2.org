.. _testing:



2. Advanced Navigation Testing Framework
========================================

**Task description** 

The ROS 2 Navigation Stack has had a focus on testing and reliability as a characteristic change from ROS 1 to ROS 2. We currently have a test coverage rate of 85% and do full system simulations in Continuous Integration (CI) to test the entire navigation system with a real robot completing real navigation tasks. Your task will be to increase the testing coverage rate to 90% (or +5% from your starting) and improve on the existing system tests to represent a more realistic environment. You will then work to make sure of that environment to actively block the robot from completing its task to simulate worst-case conditions.

**Project difficulty: Medium**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**

- Test line coverage of 90% or higher on the repository as reported by codecov (currently 85%)
- An improved simulation environment for a more realistic mobile robotics application
- Updated system tests to take advantage of that environment to fault or un-ideal cases of the stack

**Skills required**

- C++, Python3, gtest
- Gazebo, recommended experience with Gazebo plugins
- Recommended: Navigation experience

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Navigation2 <https://navigation.ros.org/>`_
- `Navigation2 Repo System Tests <https://github.com/ros-planning/navigation2/tree/main/nav2_system_tests>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.
