
.. _spinners:


7. Reduce ROS 2 Nodes and Determinism
=====================================

**Task description** 

This project is admittedly abstract to explain to someone unfamiliar with the inner-details of ROS 2 and its layers. If you're interested in working on this, you will become one of the few that truly understand the inner workings of it and be a very marketable skill. We do not expect anyone applying for this project to have that knowledge beforehand and we will help you learn the necessary items.

ROS 2 architecturally was changed before Foxy in order to ensure that any single process containing multiple ROS 2 node objects will share the same DDS participant on the network. This is important due to the overhead that each additional DDS participant has on the system.

In order for nav2 to leverage this the best, we need to adjust our usage of ROS 2 nodes and executors to further minimize the number of node objects in existance. In the early days of ROS 2 when Nav2 was being built, we were required to have many nodes in a single server in order to handle action requests and other callbacks. Now, we can make use of multi-threaded spinners, callback groups, and individual executors for specific tasks.

This project will involve identifying all of the Node objects in the stack (control+F makes this easy) and work with mentors to ensure by the end of the summer each server contains only a single node. Additionally, the behavior tree plugins should be updated to leverage callback groups to ensure that any single BT node spinning to check if any new messages are on its callback will **only** trigger its own callback by the same mechanisms.

More details about this project can be supplied if interested, but the tickets linked below also provide more context. Trust me to say this is a very achievable goal over the course of the summer and will also let you look under the hood of both Nav2 and rclcpp, giving you valuable insight future in your career (and put you in the top 10% of ROS 2 developers that know it to this degree).

**Project difficulty: Medium**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**
- Remove excess ROS 2 nodes from stack and replace with executors and multi-threaded executors
- Replace Behavior Tree node spinners with local executors to ensure deterministic execution & processing only callback groups of the current BT node
- If time allots, create a single ``main()`` function Nav2 executable composing all Nav2 processes and benchmarking improvements in performance

**Skills required**

- C++, ROS

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Github ticket <https://github.com/ros-planning/navigation2/issues/2251>`_
- `Github ticket2 <https://github.com/ros-planning/navigation2/issues/816>`_
- `Navigation2 <https://navigation.ros.org/>`_
- `Some related works <https://alyssapierson.files.wordpress.com/2018/05/pierson2018.pdf>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.
