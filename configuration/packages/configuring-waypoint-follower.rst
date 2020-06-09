.. _configuring_waypoint_follower:

Waypoint Follower
#################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_waypoint_follower

The Lifecycle Manager module implements a way of doing waypoint following using the NavigateToPose action server.
It will take in a set of ordered waypoints to follow and then try to navigate to them in order.
If a waypoint is not achievable, the ``stop_on_failure`` parameter will determine whether to continue to the next point or stop.

Parameters
**********

:stop_on_failure:

  ==== =======
  Type Default                                                   
  ---- -------
  bool true            
  ==== =======

  Description
    Whether to fail action task if a single waypoint fails. If false, will continue to next waypoint.

:loop_rate:

  ==== =======
  Type Default                                                   
  ---- -------
  int  20            
  ==== =======

  Description
    Rate to check for results from current navigation task.

Example
*******
.. code-block:: yaml

    lifecycle_manager:
      ros__parameters:
        loop_rate: 20
        stop_on_failure: false
