.. _navigation2_with_zone_parameter_filter:

Navigating with Zone Parameter Overrides
****************************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

Overview
========

This tutorial shows how to change parameters of Nav2 servers automatically when the robot enters marked zones on the map. A typical case is a snow or ice covered section of an outdoor route, or a work cell shared with people: the robot may still drive there, but it should behave more carefully - lower maximum velocity, longer backup simulation time, and so on. Keepout Filter blocks such areas entirely and Speed Filter changes only the speed limit, while ``ZoneParameterFilter`` costmap filter plugin, which will be enabled and used in this document, can change any parameter on any node. When the robot enters a zone, the filter applies the parameter setpoints configured for that zone; when the robot returns to a nominal area, it restores the declared nominal defaults.

Requirements
============

It is assumed that ROS 2, Gazebo and TurtleBot4 packages are installed or built locally. Please make sure that Nav2 project is also built locally as it was made in :ref:`build-instructions`.

Tutorial Steps
==============

1. Prepare filter mask
----------------------

As was written in :ref:`concepts`, any Costmap Filter (including Zone Parameter Filter) reads the data marked in a filter mask file. Filter mask - is the usual Nav2 2D-map distributed through PGM, PNG or BMP raster file with its metadata containing in a YAML file. The first step of the :ref:`navigation2_with_keepout_filter` tutorial explains in detail how the color lightness of mask pixels converts to ``OccupancyGrid`` values depending on the map ``mode``; please refer to it if you have not prepared a filter mask before.

For Zone Parameter Filter the ``OccupancyGrid`` value of a mask cell is the id of the state that must be active while the robot stands on that cell. Value ``0`` means the nominal state: no overrides. Values ``1`` and higher select the states declared in the filter configuration (see step 3). Since exact integer values matter here, use the ``raw`` map mode, where the ``OccupancyGrid`` value is equal to the color lightness of the pixel:

- Paint each zone with the lightness equal to its state id: ``1`` for the first zone, ``2`` for the second one, and so on.
- Fill everything else with black (lightness ``0``, the nominal state).

To make the mask, copy `depot.pgm <https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/maps/depot.pgm>`_ main map which will be used in a world simulation from a ``Nav2`` repository to a new ``depot_zones.pgm`` file and edit it in a raster graphics editor (as an example could be taken GIMP editor): paint the zones over the map areas they should cover first, then fill all remaining pixels with black.

.. note::

  In ``raw`` mode a lightness of ``101`` or higher converts to the unknown value (``-1``). On an unknown cell the filter holds the current state and prints a throttled warning, so stray light pixels will delay the switch back to nominal. Keep every pixel either at ``0`` or at a declared state id. When the robot leaves the mask coverage entirely, the filter resets to the nominal state.

Like all other maps, the filter mask should have its own YAML metadata file. Copy `depot.yaml <https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/maps/depot.yaml>`_ to ``depot_zones.yaml``, then point the ``image`` field to the new mask and set the ``raw`` mode:

.. code-block:: yaml

  image: depot_zones.pgm
  mode: raw

Save ``depot_zones.yaml`` and the new filter mask is ready to use.

2. Configure Costmap Filter Info Publisher Server
-------------------------------------------------

Each costmap filter reads incoming meta-information (such as filter type or data conversion coefficients) in messages of ``nav2_msgs/CostmapFilterInfo`` type. These messages are being published by `Costmap Filter Info Publisher Server <https://github.com/ros-navigation/navigation2/tree/main/nav2_map_server/src/costmap_filter_info>`_. The server is running as a lifecycle node. ``nav2_msgs/CostmapFilterInfo`` messages are going in a pair with ``OccupancyGrid`` filter mask topic, therefore along with Costmap Filter Info Publisher Server there should be enabled a new instance of Map Server configured to publish the filter mask. Parameters of both servers are listed at :ref:`configuring_map_server` page. The example of ``params_file`` for them:

.. code-block:: yaml

  costmap_filter_info_server:
    ros__parameters:
      use_sim_time: true
      type: 4
      filter_info_topic: "/costmap_filter_info"
      mask_topic: "/zone_filter_mask"
      base: 0.0
      multiplier: 1.0

  filter_mask_server:
    ros__parameters:
      use_sim_time: true
      frame_id: "map"
      topic_name: "/zone_filter_mask"
      yaml_filename: "depot_zones.yaml"

Note, that:

 - For Zone Parameter Filter the ``type`` of costmap filter should be set to ``4``.
 - Filter mask topic name should be the equal for ``mask_topic`` parameter of Costmap Filter Info Publisher Server and ``topic_name`` parameter of Map Server.
 - ``base`` and ``multiplier`` are not used by Zone Parameter Filter: mask values are taken directly as state ids, without a linear conversion. Keep them at ``0.0`` and ``1.0``; other values produce a warning at startup.

Both servers could be run as lifecycle nodes with a standalone launch-file like the one below (the same shape the `costmap filters demo <https://github.com/ros-navigation/navigation2_tutorials/tree/rolling/nav2_costmap_filters_demo>`_ from ``navigation2_tutorials`` uses for the other filters):

.. code-block:: python

  from launch import LaunchDescription
  from launch.actions import DeclareLaunchArgument
  from launch.substitutions import LaunchConfiguration
  from launch_ros.actions import Node


  def generate_launch_description() -> LaunchDescription:
      params_file = LaunchConfiguration('params_file')
      mask_yaml_file = LaunchConfiguration('mask')

      declare_params_file_cmd = DeclareLaunchArgument(
          'params_file',
          description='Full path to the ROS 2 parameters file for both servers')

      declare_mask_yaml_file_cmd = DeclareLaunchArgument(
          'mask',
          description='Full path to filter mask yaml file to load')

      start_filter_mask_server_cmd = Node(
          package='nav2_map_server',
          executable='map_server',
          name='filter_mask_server',
          output='screen',
          parameters=[params_file, {'yaml_filename': mask_yaml_file}])

      start_costmap_filter_info_server_cmd = Node(
          package='nav2_map_server',
          executable='costmap_filter_info_server',
          name='costmap_filter_info_server',
          output='screen',
          parameters=[params_file])

      start_lifecycle_manager_cmd = Node(
          package='nav2_lifecycle_manager',
          executable='lifecycle_manager',
          name='lifecycle_manager_costmap_filters',
          output='screen',
          parameters=[{'use_sim_time': True},
                      {'autostart': True},
                      {'node_names': ['filter_mask_server',
                                      'costmap_filter_info_server']}])

      ld = LaunchDescription()
      ld.add_action(declare_params_file_cmd)
      ld.add_action(declare_mask_yaml_file_cmd)
      ld.add_action(start_filter_mask_server_cmd)
      ld.add_action(start_costmap_filter_info_server_cmd)
      ld.add_action(start_lifecycle_manager_cmd)
      return ld

3. Enable Zone Parameter Filter
-------------------------------

Costmap Filters are Costmap2D plugins. You can enable the ``ZoneParameterFilter`` plugin by adding ``zone_params`` to the ``filters`` parameter in ``nav2_params.yaml``. Zone Parameter Filter does not change the costmap values: it only tracks which mask cell the robot stands on and issues parameter updates on state changes. One instance in the ``global_costmap`` is enough; a second instance in ``local_costmap`` would send every update twice.

The filter configuration consists of three parts: the states, their setpoints and the nominal defaults.

``states`` lists the state names. Each state declares an ``id`` - the mask cell value that selects it:

.. code-block:: yaml

  zone_params:
    plugin: "nav2_costmap_2d::ZoneParameterFilter"
    filter_info_topic: "/costmap_filter_info"
    states: ["snow_zone", "work_zone"]
    snow_zone:
      id: 1
    work_zone:
      id: 2

Valid ids are ``1`` through ``255``; ``0`` is reserved for the nominal state. A mask published by Map Server carries values up to ``100``, so in practice ids stay in that range. Every non-zero value present in the mask must have a state declared for it: the filter treats an undeclared positive value as a configuration error and throws.

Each state declares its ``setpoints`` - the parameters to override while its zone is active. A setpoint names the target ``node``, the ``parameter`` on that node and the ``value`` to set. The value may be of any parameter type: double, integer, boolean, string or list. A setpoint with an empty ``node`` or ``parameter``, or with no ``value``, is reported at startup and skipped:

.. code-block:: yaml

  zone_params:
    ...
    snow_zone:
      id: 1
      setpoints: ["slow_fwd", "long_backup"]
      slow_fwd:
        node: "controller_server"
        parameter: "FollowPath.max_vel_x"
        value: 0.15
      long_backup:
        node: "behavior_server"
        parameter: "backup.simulate_ahead_time"
        value: 2.5

``nominal_defaults`` declares the baseline values, in the same ``node`` / ``parameter`` / ``value`` form, that the filter restores when the robot returns to a ``0`` area or leaves the mask. Each entry lives in the ``nominal_defaults`` namespace. Since ``nominal_defaults`` is already the name of the list, the entry keys are written in dotted form (``nominal_defaults.fwd_speed:``): Declare a nominal entry for every parameter your states touch: the filter warns at startup about each state setpoint without a matching entry, and the reset will not restore that parameter:

.. code-block:: yaml

  zone_params:
    ...
    nominal_defaults: ["fwd_speed", "backup_time"]
    nominal_defaults.fwd_speed:
      node: "controller_server"
      parameter: "FollowPath.max_vel_x"
      value: 0.26
    nominal_defaults.backup_time:
      node: "behavior_server"
      parameter: "backup.simulate_ahead_time"
      value: 2.0

The resulting configuration:

.. code-block:: yaml

  global_costmap:
    global_costmap:
      ros__parameters:
        ...
        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        filters: ["zone_params"]
        ...
        zone_params:
          plugin: "nav2_costmap_2d::ZoneParameterFilter"
          enabled: True
          filter_info_topic: "/costmap_filter_info"
          states: ["snow_zone", "work_zone"]
          snow_zone:
            id: 1
            setpoints: ["slow_fwd", "long_backup"]
            slow_fwd:
              node: "controller_server"
              parameter: "FollowPath.max_vel_x"
              value: 0.15
            long_backup:
              node: "behavior_server"
              parameter: "backup.simulate_ahead_time"
              value: 2.5
          work_zone:
            id: 2
            setpoints: ["crawl_fwd"]
            crawl_fwd:
              node: "controller_server"
              parameter: "FollowPath.max_vel_x"
              value: 0.10
          nominal_defaults: ["fwd_speed", "backup_time"]
          nominal_defaults.fwd_speed:
            node: "controller_server"
            parameter: "FollowPath.max_vel_x"
            value: 0.26
          nominal_defaults.backup_time:
            node: "behavior_server"
            parameter: "backup.simulate_ahead_time"
            value: 2.0

On a transition between two zones the filter first resets the parameters set by the previous state but not by the new one back to their nominal defaults, then applies the new state's setpoints, batched per target node. The updates are issued asynchronously, so the costmap update loop is not blocked. An update that fails or is rejected by the target node makes the filter throw instead of continuing silently. Every state change is also published as a ``std_msgs/UInt8`` message on the topic set by the ``state_event_topic`` parameter (default: ``zone_filter_state``).

Full list of parameters supported by ``ZoneParameterFilter`` are listed at :ref:`zone_parameter_filter` page.

4. Run Nav2 stack
-----------------

Run Nav2 with the modified parameters file:

.. code-block:: bash

  ros2 launch nav2_bringup tb4_simulation_launch.py params_file:=/path/to/nav2_params.yaml

Then in a second terminal launch both filter servers from step 2:

.. code-block:: bash

  ros2 launch /path/to/zone_filter_info.launch.py params_file:=/path/to/zone_filter_server_params.yaml mask:=/path/to/depot_zones.yaml

Check the costmap log for the ``ZoneParameterFilter: Received filter info`` and ``ZoneParameterFilter: Received filter mask`` lines: the filter starts working right after both arrive.

5. Verify zone transitions
--------------------------

Echo the state event topic. The default ``zone_filter_state`` name is resolved relative to the costmap's parent namespace, so in the single-robot setup used here it appears at the root:

.. code-block:: bash

  ros2 topic echo /zone_filter_state

Set a Nav2 goal in RViz so that the route passes through a painted zone. When the robot enters the ``snow_zone`` area, the topic shows ``data: 1`` and the setpoints are applied. Since the updates travel through the target node's parameter service, they take effect shortly after the transition, not in the same instant:

.. code-block:: bash

  $ ros2 param get /controller_server FollowPath.max_vel_x
  Double value is: 0.15

When the robot drives out to a black area, the topic shows ``data: 0`` and the nominal defaults are restored:

.. code-block:: bash

  $ ros2 param get /controller_server FollowPath.max_vel_x
  Double value is: 0.26

The costmap log follows the transitions as well::

  ZoneParameterFilter: Entered state 1 (reset 0 N-only parameter(s); applied 2 parameter(s) across 2 node(s)).
  ZoneParameterFilter: Entered state 0 (reset to nominal).
