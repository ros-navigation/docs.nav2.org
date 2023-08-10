.. _configuring_collision_monitor:

Nav2 collision monitor
######################

Source code and ``README`` with design, explanations, and metrics can be found on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_collision_monitor

The ``nav2_collision_monitor`` package contains nodes providing an additional level of robot safety, namely the Collision Detector and the Collision Monitor.
The Collision Detector is responsible for detecting obstacles in a certain static region or in the robot's path and reporting them/ 
The Collision Monitor also detects obstacles and additionally limits the robot's velocity to avoid collisions with these obstacles.

Provided Nodes
****************
The nodes listed below are inside the ``nav2_collision_monitor`` package. See the pages for individual configuration information.

.. toctree::
  :maxdepth: 1

  collision_monitor/configuring-collision-monitor-node.rst
  collision_monitor/configuring-collision-detector-node.rst

