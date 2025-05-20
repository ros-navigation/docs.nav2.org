.. _kilted_migration:

Kilted to L-turtle
##################

Moving from ROS 2 Kilted to L-Turtle, a number of stability improvements were added that we will not specifically address here.


Removed Parameter action_server_result_timeout
**********************************************

Removed the parameter ``action_server_result_timeout`` from all action servers after resolution within ``rcl`` and ``rclcpp`` to address early goal removal.
This is not longer required to be set.
