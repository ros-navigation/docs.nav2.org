.. _configuring_behavior_tree:

Configuring the behavior tree XML
#################################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_lifecycle_manager

Behavior Tree Plugins
*********************

Actions
=======

  .. toctree::
    :maxdepth: 1

    bt-plugins/actions/Wait.rst
    bt-plugins/actions/Spin.rst
    bt-plugins/actions/BackUp.rst
    bt-plugins/actions/ComputePathToPose.rst
    bt-plugins/actions/FollowPath.rst
    bt-plugins/actions/NavigateToPose.rst
    bt-plugins/actions/ClearEntireCostmap.rst
    bt-plugins/actions/ReinitializeGlobalLocalization.rst


Conditions
==========

  .. toctree::
    :maxdepth: 1

    bt-plugins/conditions/GoalReached.rst
    bt-plugins/conditions/TransformAvailable.rst

Controls
========

  .. toctree::
    :maxdepth: 1

    bt-plugins/RecoveryNode.rst

Decorators
============

  .. toctree::
    :maxdepth: 1

    bt-plugins/RateController.rst
    bt-plugins/DistanceController.rst

Example
*******
.. code-block:: yaml

    lifecycle_manager:
      ros__parameters:
        autostart: true
        node_names: ['controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower']
