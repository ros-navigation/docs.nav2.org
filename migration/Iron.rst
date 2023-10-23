.. _iron_migration:

Iron to Jazzy
##############

Moving from ROS 2 Iron to Jazzy, a number of stability improvements were added that we will not specifically address here.

Introduce a new Multi-Robot Bringup Launch
******************************************

`PR #3572 <https://github.com/ros-planning/navigation2/pull/3572>`_ introduces a new way of bringup tb3 multi-robot that names as ``cloned_tb3_simulation_launch.py`` for simulation. ``cloned_tb3_simulation_launch.py`` enables to bring up multiple robots with same parameter that described in ``nav2_multirobot_param_all.yaml``. And multiple robots are separeted by namespaces which are given as a Launch Arguments.
Existing ``multi_tb3_simulation_launch.py`` which was utilized in previous is replaced with ``unique_tb3_simulation_launch.py``, allowing for multiple unique robot instances utilizing ``nav2_multirobot_params_<N>.yaml`` configuration files.


New option for the Voxel and Obstacle Layers
********************************************
`PR #3612 <https://github.com/ros-planning/navigation2/pull/3612>`_ adds a new MaxWithoutUnknownOverwrite option to combination_method parameter in Voxel and Obstacle Layers. This can be used to make sure that the static map is the dominant source of information, and
easily prevent the robot to go through places that are not present in the static map.

Changes to MPPI Goal Critic
***************************

The MPPI Goal critic's formulation is changed to better keep up with speed on approach to goal instead of preemptively slowing too significantly. It also allows you to better use the weight to adjust the degree at which it slows more naturally. This change involves adjusting the ``threshold_to_consider`` to be the same as your prediction horizon (e.g. samples * dt * max speed) for both the goal critic and path follower critic to have a good hand-off between them without deceleration. 

Changes to MPPI Path Angle Critic
*********************************

MPPI's Path Angle critic now has a ``mode`` setting to adjust behavior depending on robot's desired behavioral traits. Previously, it penalized path orientations that deviated far the the robot's forward orientation to turn the robot towards sharp changes in the path. This is still default (``mode: 0``), but other modes now exist too. 

``mode: 1`` sets the penalization of path's relative directions by either forward orientation or the opposite for reversing to allow for true bidirectional motion when one way or another is not preferable for a symmetric robot. This uses only the path's relative points to the robot to decide which direction to incentivize. 

``mode: 2`` instead uses the path's orientations when a feasible path is given from the Smac Planners or the Smoother server's algorithms. This way, the globally planned orientations are followed rather than the based solely on the path's relative points. This is useful for non-circular robots in highly confined settings where there may be restricted opportunities to change directions so following the global path's orientation are required to end in the orientation you require.


Changes to MPPI Path Handling For Directionality
************************************************

MPPI's Path Align Critic and Path Handler object now have options to utilize the path's orientation information to force the controller to change directions when and only when requested by a feasible planner. When ``enforce_path_inversion`` is ``true``, the path handler will prune the path to the first time the directions change to force the controller to plan to the inversion point and then be set the rest of the path, once in tolerance. The Path Align critic also contains a parameter ``use_path_orientations``  which can be paired with it to incentivize aligning the path containing orientation information to better attempt to achieve path inversions where requested **and** not do them when not requested.

See MPPI's configuration guide for complete information.

Move Error Code Enumerations
****************************

`PR #3693 <https://github.com/ros-planning/navigation2/pull/3693>`_ moves the enumeration codes from the goal to the result section. 

Substitution in parameter file
******************************

Enabled substitution in parameter file. For example, you can write the following

.. code-block:: yaml

    bt_navigator:
      ros__parameters:
        default_nav_to_pose_bt_xml: $(find-pkg-share my_package)/behavior_tree/my_nav_to_pose_bt.xml

For more information about substitutions syntax, see `here <https://docs.ros.org/en/rolling/How-To-Guides/Launch-files-migration-guide.html#substitutions>`_

Allow Behavior Server Plugins to Access The Action Result
*********************************************************

`PR #3704 <https://github.com/ros-planning/navigation2/pull/3704>`_ allows behavior servers plugins to access and modify the action result.

Smac Planner Debug Param Name Change
************************************

``debug_visualizations`` replaces ``viz_expansions`` parameter in Hybrid-A* to reflect the new inclusion of footprint debug information being published as well.

New node in nav2_collision_monitor: Collision Detector
******************************************************

In this `PR #3693 <https://github.com/ros-planning/navigation2/pull/3500>`_ A new node was introduced in the nav2_collision_monitor: Collision Detector. 
It works similarly to the Collision Monitor, but does not affect the robot's velocity. It will only inform that data from the configured sources has been detected within the configured polygons via message to the ``collision_detector_state`` topic that might be used by any external module (e.g. switching LED or sound alarm in case of collision).

Binary Filter able to change boolean ros parameters
****************************************************

`PR #3796 <https://github.com/ros-planning/navigation2/pull/3796>`_ modifies the binary filter to make it able to change boolean ros parameters. 