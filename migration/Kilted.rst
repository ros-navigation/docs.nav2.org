.. _kilted_migration:

Kilted to L-turtle
##################

Moving from ROS 2 Kilted to L-Turtle, a number of stability improvements were added that we will not specifically address here.


Removed Parameter action_server_result_timeout
**********************************************

Removed the parameter ``action_server_result_timeout`` from all action servers after resolution within ``rcl`` and ``rclcpp`` to address early goal removal.
This is not longer required to be set.

Added Corner Smoothing functionality to route_server
****************************************************

In `PR #5226 <https://github.com/ros-navigation/navigation2/pull/5226>`_ the ability to stitch two successive edges in ``route_server`` with a smooth circular arc has been added. Below is an example of two successive edges forming a corner being smoothed with a radius of one. The red lines are the edges of the route graph and the green line is the resultant path that can be used by a local planner.

.. image:: images/smoothing.png

New parameters include ``smooth_corners`` which enable or disable corner smoothing and ``smoothing_radius`` which specifies the radius of the corner to fit to a corner. The tangents of the starting and ending points of the circular arc will match the tangent of the edges that form the corner. In the event that two edges are basically straight, no corner arc is added and regular linear interpolation is done. In addition to that, if the corner arc requires a starting point and ending point that's longer than the edge lengths, then it will not add a corner arc.
