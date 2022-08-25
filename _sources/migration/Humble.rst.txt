.. _humble_migration

Humble to Iron
##############

Moving from ROS 2 Humble to Iron, a number of stability improvements were added that we will not specifically address here.

Added Collision Monitor
***********************
`PR 2982 <https://github.com/ros-planning/navigation2/pull/2982>`_ adds new safety layer operating independently of Nav2 stack which ensures the robot to control the collisions with near obstacles, obtained from different sensors (LaserScan, PointCloud, IR, Sonars, etc...). See :ref:`configuring_collision_monitor` for more details. It is not included in the default bringup batteries included from ``nav2_bringup``.

Removed use_sim_time from yaml
******************************
`PR #3131 <https://github.com/ros-planning/navigation2/pull/3131>`_ makes it possible to set the use_sim_time parameter from the launch file for multiple nodes instead of individually via the yaml files. If using the Nav2 launch files, you can optionally remove the use_sim_time parameter from your yaml files and set it via a launch argument.