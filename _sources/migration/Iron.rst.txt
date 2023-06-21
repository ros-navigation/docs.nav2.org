.. _iron_migration:

Iron to Jazzy
##############

Moving from ROS 2 Iron to Jazzy, a number of stability improvements were added that we will not specifically address here.

New option for the Voxel and Obstacle Layers
********************************************
`PR #3612 <https://github.com/ros-planning/navigation2/pull/3612>`_ adds a new MaxWithoutUnknownOverwrite option to combination_method parameter in Voxel and Obstacle Layers. This can be used to make sure that the static map is the dominant source of information, and
easily prevent the robot to go through places that are not present in the static map.

