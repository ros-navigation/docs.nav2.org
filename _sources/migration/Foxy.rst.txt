.. _foxy_migration:

Foxy to Galactic
################

Moving from ROS 2 Foxy to Galactic, a number of stability improvements were added that we will not specifically address here.

NavigateToPose BT-node Interface Changes
****************************************

The NavigateToPose input port has been changed to PoseStamped instead of Point and Quaternion.

See :ref:`bt_navigate_to_pose_action` for more information.

BackUp BT-node Interface Changes
********************************

The ``backup_dist`` and ``backup_speed`` input ports should both be positive values indicating the distance to go backward respectively the speed with which the robot drives backward.

BackUp Recovery Interface Changes
*********************************

``speed`` in a backup recovery goal should be positive indicating the speed with which to drive backward.
``target.x`` in a backup recovery goal should be positive indicating the distance to drive backward.
In both cases negative values are silently inverted.

Costmap Filters
***************

A new concept interacting with spatial-dependent objects called "Costmap Filters" appeared in Galactic (more information about this concept could be found at :ref:`concepts` page). Costmap filters are acting as a costmap plugins. In order to make a filtered costmap and change robot's behavior in annotated areas, filter plugin reads the data came from filter mask. Then this data is being linearly transformed into feature map in a filter space. It could be passability of an area, maximum speed limit in m/s, robot desired direction in degrees or anything else. Transformed feature map along with the map/costmap, sensors data and current robot position is used in plugin's algorithms to make required updates in the resulting costmap and robot's behavor.

Architecturally, costmap filters consists from ``CostmapFilter`` class which is a basic class incorporating much common of its inherited filter plugins:

- ``KeepoutFilter``: keep-out/safety zones filter plugin.
- ``SpeedFilter``: slow/speed-restricted areas filter.
- Preferred lanes in industries. This plugin is covered by ``KeepoutFilter`` (see discussion in `corresponding PR <https://github.com/ros-planning/navigation2/issues/1522>`_ for more details).

Each costmap filter subscribes to filter info topic (publishing by `Costmap Filter Info Publisher Server <https://github.com/ros-planning/navigation2/tree/main/nav2_map_server/src/costmap_filter_info>`_) having all necessary information for loaded costmap filter and filter mask topic.

High-level design of this concept could be found `here <https://github.com/ros-planning/navigation2/tree/main/doc/design/CostmapFilters_design.pdf>`_. The functionality of costmap filters is being disscussed in `the ticket #1263 <https://github.com/ros-planning/navigation2/issues/1263>`_ and carried out by `PR #1882 <https://github.com/ros-planning/navigation2/pull/1882>`_. The following tutorial: :ref:`navigation2_with_keepout_filter` will help to easily get involved with ``KeepoutFilter`` functionality.
