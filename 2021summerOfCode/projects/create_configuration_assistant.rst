:orphan:

.. _create_moveit_analog:

1. Create a Configuration Assistant (Analog to MoveIt)
======================================================

**Task description** 

`Moveit <https://moveit.ros.org/>`_ has long has a QT `configuration assistant <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html>`_. This setup assistent helps the user configure their UDRF and needs to setup MoveIt configuration files.

A configuration assistant could be extremely beneficial to Navigation2 users as a way to minimize friction. We should provide a gui tool to cover the following configurations:

- the broad strokes with the costmap, with a visualizer to show the user what it will look like
- Select configurable costmap layers
- Select recovery behavior parameters
- URDF, footprint, and frame selection to make sure the options comply with standards, planner, and controller
- Set minimum and maximum speed and other kinematic parameters
- Select from a dropdown of possible planners and controllers
- Helpful notes throughout the prompts to aid in selecting appropriate parameters
- Selecting at behavior tree
- @steve please add more specific options

After the items are configured, there should be a preview to see how the parameters effect the robot.

**Project difficulty: High**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Contact information for the cooperating mentor (optional):  juzhenatpku@gmail.com**

**Project output requirements**

- A QT based GUI configuration assistant that support the parameters listed above
- A preview panel to display the parameters' effection on the robot

**Skills required**

- C++, Python3, QT framework
- JSON/XML parsing
- 3D programming (maybe needed in the preview)
- Recommended: Gazebo simulation, ROS, and Navigation experience

**List of relevant open source software repositories and refs** 

- `QT <https://www.qt.io/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Original github issue page <https://github.com/ros-planning/navigation2/issues/1721>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.