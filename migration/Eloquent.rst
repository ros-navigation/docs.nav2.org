.. _eloquent_migration:

Eloquent to Foxy
################

Moving from ROS 2 Eloquent to Foxy, a number of stability improvements were added that we will not specifically address here.
We will specifically mention, however, the reduction in terminal noise.
TF2 transformation timeout errors and warnings on startup have been largely removed or throttled to be more tractable.
Additionally, message filters filling up resulting in messages being dropped were resolved in costmap 2d.

General
*******

The lifecycle manager was split into 2 unique lifecycle managers.
They are the ``navigation_lifecycle_manager`` and ``localization_lifecycle_manager``.
This gives each process their own manager to allow users to switch between SLAM and localization without effecting Navigation.
It also reduces the redundant code in ``nav2_bringup``.

The lifecycle manager also now contains ``Bond`` connections to each lifecycle server.
This means that if a server crashes or exits, the lifecycle manager will be constantly checking and transition down its lifecycle nodes for safety.
This acts as a watchdog during run-time to complement the lifecycle manager's transitioning up and down from active states. See `this PR for details <https://github.com/ros-planning/navigation2/pull/1894>`_.

A fix to the BT navigator was added to remove a rare issue where it may crash due to asynchronous issues.
As a result, a behavior tree is created for each navigation request rather than resetting an existing tree.
The creation of this tree will add a small amount of latency.
Proposals to reduce this latency will be considered before the next release.

Server Updates
**************
All plugin servers (controller, planner, recovery) now supports the use of multiple plugins.
This can be done by loading a map of plugins, mapping the name of the plugin to its intended use-case.
Each server defines a parameter where the list of names for the plugins to be loaded can be defined.

+-----------------------+------------------------+
|      Server Name      |    Plugin Parameter    |
+=======================+========================+
| Controller Server     | progress_checker_plugin|
+-----------------------+------------------------+
| Controller Server     | goal_checker_plugin    |
+-----------------------+------------------------+
| Controller Server     | controller_plugins     |
+-----------------------+------------------------+
| Planner Server        | planner_plugins        |
+-----------------------+------------------------+
| Recovery Server       | recovery_plugins       |
+-----------------------+------------------------+
| Costmap Node          | plugins                |
+-----------------------+------------------------+

The type of plugin to be mapped to a particular name has to be defined using the ``plugin`` parameter in the plugin namespace for each name defined in the server plugin list.
Each name in the plugin parameter list is expected to have the ``plugin`` parameter defined.

An example: ``controller_server`` defines the parameter ``controller_plugins`` where a list of plugin names can be defined:

.. code-block:: yaml

    controller_server:
      ros__parameters:
        controller_plugins: ["FollowPath", "DynamicFollowPath"]
        FollowPath:
          plugin: "dwb_core/DWBLocalPlanner"
          max_vel_x: 0.26
        DynamicFollowPath:
          plugin: "teb_local_planner/TEBLocalPlanner"
          max_vel_x: 0.5


``FollowPath`` controller is of type ``dwb_local_planner/DWBLocalPlanner`` and ``DynamicFollowPath`` of type ``teb_local_planner/TEBLocalPlanner``.
Each plugin will load the parameters in their namespace, e.g. ``FollowPath.max_vel_x``, rather than globally in the server namespace.
This will allow multiple plugins of the same type with different parameters and reduce conflicting parameter names.

DWB Contains new parameters as an update relative to the ROS 1 updates, `see here for more information <https://github.com/ros-planning/navigation2/pull/1501>`_.
Additionally, the controller and planner interfaces were updated to include a ``std::string name`` parameter on initialization.
This was added to the interfaces to allow the plugins to know the namespace it should load its parameters in.
E.g. for a controller to find the parameter ``FollowPath.max_vel_x``, it must be given its name, ``FollowPath`` to get this parameter.
All plugins will be expected to look up parameters in the namespace of its given name.

New Plugins
***********

Many new behavior tree nodes were added.
These behavior tree nodes are now BT plugins and dynamically loadable at run-time using behavior tree cpp v3.
The default behavior trees have been upgraded to stop the recovery behaviours and trigger a replanning when the navigation goal is preempted.
See ``nav2_behavior_tree`` for a full listing, or :ref:`plugins` for the current list of behavior tree plugins and their descriptions.
These plugins are set as default in the ``nav2_bt_navigator`` but may be overridden by the ``bt_plugins`` parameter to include your specific plugins.

Original GitHub tickets:

- `DistanceController <https://github.com/ros-planning/navigation2/pull/1699>`_
- `SpeedController <https://github.com/ros-planning/navigation2/pull/1744>`_
- `GoalUpdatedCondition <https://github.com/ros-planning/navigation2/pull/1712>`_
- `DistanceTraveledCondition <https://github.com/ros-planning/navigation2/pull/1705>`_
- `TimeExpiredCondition <https://github.com/ros-planning/navigation2/pull/1705>`_
- `UpdateGoal <https://github.com/ros-planning/navigation2/pull/1859>`_
- `TruncatePath <https://github.com/ros-planning/navigation2/pull/1859>`_
- `IsBatteryLowCondition <https://github.com/ros-planning/navigation2/pull/1974>`_
- `ProgressChecker <https://github.com/ros-planning/navigation2/pull/1857>`_
- `GoalChecker <https://github.com/ros-planning/navigation2/pull/1857>`_

Map Server Re-Work
******************

``map_saver`` was re-worked and divided into 2 parts: CLI and server.
CLI part is a command-line tool that listens incoming map topic, saves map once into a file and finishes its work. This part is remained to be almost untouched: CLI executable was renamed from ``map_saver`` to ``map_saver_cli`` without changing its functionality.
Server is a new part. It spins in the background and can be used to save map continuously through a ``save_map`` service. By each service request it tries to listen incoming map topic, receive a message from it and write obtained map into a file.

``map_server`` was dramatically simplified and cleaned-up. ``OccGridLoader`` was merged with ``MapServer`` class as it is intended to work only with one ``OccupancyGrid`` type of messages in foreseeable future.

Map Server now has new ``map_io`` dynamic library. All functions saving/loading ``OccupancyGrid`` messages were moved from ``map_server`` and ``map_saver`` here. These functions could be easily called from any part of external ROS 2 code even if Map Server node was not started.

``map_loader`` was completely removed from ``nav2_util``. All its functionality already present in ``map_io``. Please use it in your code instead.

Please refer to the `original GitHub ticket <https://github.com/ros-planning/navigation2/issues/1010>`_ and `Map Server README <https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md>`_ for more information.


New Particle Filter Messages
****************************

New particle filter messages for particle clouds were added to include the particle weights along with their poses.
``nav2_msgs/Particle`` defines a single particle with a pose and a weight in a particle cloud.
``nav2_msgs/ParticleCloud`` defines a set of particles, each with a pose and a weight.

``AMCL`` now publishes its particle cloud as a ``nav2_msgs/ParticleCloud`` instead of a ``geometry_msgs/PoseArray``.

`See here for more information. <https://github.com/ros-planning/navigation2/pull/1677>`_


Selection of Behavior Tree in each navigation action
****************************************************

The ``NavigateToPose`` action allows now to select in the action request the behavior tree to be used by ``bt_navigator`` for carrying out the navigation action through the ``string behavior_tree`` field. This field indicates the absolute path of the xml file that will be used to use to carry out the action. If no file is specified, leaving this field empty, the default behavior tree specified in the ``default_bt_xml_filename parameter`` will be used.

This functionality has been discussed in `the ticket #1780 <https://github.com/ros-planning/navigation2/issues/1780>`_, and carried out in `the pull request #1784 <https://github.com/ros-planning/navigation2/pull/1784>`_.


FollowPoint Capability
**********************

A new behavior tree ``followpoint.xml`` has added. This behavior tree makes a robot follow a dynamically generated point, keeping a certain distance from the target. This can be used for moving target following maneuvers.

This functionality has been discussed in `the ticket #1660 <https://github.com/ros-planning/navigation2/issues/1660>`_, and carried out in `the pull request #1859 <https://github.com/ros-planning/navigation2/issues/1859>`_.

New Costmap Layer
*****************
The range sensor costmap has not been ported to navigation2 as ``nav2_costmap_2d::RangeSensorLayer"``. It uses the same
probabilistic model as the `ROS1 <http://wiki.ros.org/range_sensor_layer>`_ layer as well as much of the
same interface. Documentation on parameters has been added to docs/parameters and the navigation.ros.org under ``Configuration Guide``.
