
.. _twist:


8. Convert Twist to TwistStamped in Ecosystem and Run-Time Configuration
========================================================================

**Task description** 

This project is comprised of 2 smaller projects that can be easily worked on in parallel.

Subproject A: Convert Twist to TwistStamped in Ecosystem

The aim of this project is to identify places in the ROS 2 ecosystem that make use of ``Twist`` in the context of ``cmd_vel`` coming out of Navigation. This includes things like Nav2, ROS 2 Control, Gazebo ROS Plugins, Yuk's Velocity Filter, major robot drivers, etc. A set of previously identified places is shown in the ticket linked below.

Once you've created a list of places in the ecosystem where it is used, your project will be to submit PRs on their ROS 2 branches to change the interface to make use of a ``TwistStamped`` instead of a ``Twist``.


Subproject B: Run-time Reconfiguration of Parameters

In the meantime while you're waiting for PRs to be merged or blocked by reviews on converting all of the ecosystems ``cmd_vel`` use of ``Twist`` to ``TwistStamped``, your project will be to enable run-time reconfiguration of the major parameters in Nav2. In ROS 2 this is done via the parameter change event callbacks. See tickets below for a list of plugins or servers needing dynamically reconfigurable parameter support added. 


**Project difficulty: Medium**

**Project community mentor: Steve Macenski** `@SteveMacenski <https://github.com/SteveMacenski>`_

**Mentor contact details: [See link above, link in GitHub profile description]**

**Project output requirements**
- Convert all the major ecosystem projects into TwistStamped
- Enable run-time reconfiguration of the remaining plugins and servers in Nav2 missing
- Ensure that reconfiguration is thread-safe by using locks, atomic variables, or callback groups

**Skills required**

- C++, Python3
- ROS 2

**List of relevant open source software repositories and refs** 

- `ROS <https://www.ros.org/>`_
- `Gazebo Simulator <http://gazebosim.org/>`_
- `Github ticket <https://github.com/ros-planning/navigation2/issues/956>`_
- `Github ticket2 <https://github.com/ros-planning/navigation2/issues/1594>`_
- `Navigation2 <https://navigation.ros.org/>`_
- `Some related works <https://alyssapierson.files.wordpress.com/2018/05/pierson2018.pdf>`_

**Licensing**
- All contributions will be under the Apache 2.0 license.
- No other CLA's are required.
