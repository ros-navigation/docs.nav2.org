
.. _assisted_teleop:

3. Assisted Teleop
==================

**Task description** 

In mobile robot and autonomous vehicle navigation, there are situations where a human driver is required to intervene to get the vehicle out of a sticky situation. This can be both as a backup in case of autonomy failure as well as the primary function of the robot (e.g. telepresence robots). 

This project's aim is to create an assisted teleop feature in Nav2 by means of a new behavior tree configuration file (the file that defines the flow of information for the navigation task) and potentially new plugins. This feature should make sure to use the local costmap and/or sensor data in order to avoid obstacles and take position and/or velocity commands to attempt to follow.

An example application of this is a telepresence robot, where a human driver is driving the robot through a space to visit in an office building or hospital. Another example would be an autonomous delivery robot stuck requiring a human driver to navigate it back into an open space for the robot to continue its task.

This will be an excellent chance to make a substantial new feature in the Nav2 system to be used by hundreds of robots in the future. This project could also be a good candidate for a ROSCon talk in future events.

**Project difficulty: Medium**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**
- Integrations of an assisted teleop feature in Nav2 as either a set of plugins and/or a behavior tree configuration
- Robot can successfully navigate a space by a user teleop without collision
- If time allots, work on tuning / adding new critics to the DWB local planner to improve safety of its performance for users out of the box

**Skills required**

- C++, XML, ROS
- Mobile robot navigation experience
- Recommended: Gazebo simulation, ROS navigation, Behavior trees

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Github ticket <https://github.com/ros-planning/navigation2/issues/2226>`_
- `Navigation2 <https://navigation.ros.org/>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.
