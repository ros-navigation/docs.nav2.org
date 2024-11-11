.. _configuring_collision_monitor:

Collision Monitor
#################

Source code and ``README`` with design, explanations, and metrics can be found on Github_.

.. _Github: https://github.com/ros-navigation/navigation2/tree/main/nav2_collision_monitor

The ``nav2_collision_monitor`` package contains nodes providing an additional level of robot safety, namely the Collision Monitor and the Collision Detector.
The Collision Monitor is a node providing an additional level of robot safety. It performs several collision avoidance related tasks using incoming data from the sensors, bypassing the costmap and trajectory planners, to monitor for and prevent potential collisions at the emergency-stop level.
The Collision Detector works similarly to the Collision Monitor, but does not affect the robot's velocity. It will only inform that data from the configured sources has been detected within the configured polygons via a message to a topic.

Provided Nodes
****************
The nodes listed below are inside the ``nav2_collision_monitor`` package. See the pages for individual configuration information.

.. toctree::
  :maxdepth: 1

  collision_monitor/configuring-collision-monitor-node.rst
  collision_monitor/configuring-collision-detector-node.rst

