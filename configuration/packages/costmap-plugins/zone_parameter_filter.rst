.. _zone_parameter_filter:

Zone Parameter Filter Parameters
================================

Zone Parameter Filter is a Costmap Filter that sets ROS parameters on other nodes based on where the robot is on the filter mask. Each mask value selects a declared state; each state carries setpoints (``node``, ``parameter`` and ``value``) that are applied when the robot enters a zone of that value, for example a lower ``FollowPath.max_vel_x`` inside a snow zone. Leaving all zones, or entering a zone with mask value ``0``, restores the declared ``nominal_defaults``. Filter mask published by Map Server, goes in a pair with filter info topic published by Costmap Filter Info Server. The ``type`` field of the ``CostmapFilterInfo`` message must be ``4``; ``base`` and ``multiplier`` are unused by this filter and warn when not left at ``0.0`` and ``1.0``.


On a transition between two states, parameters set by the previous state but not by the new one are first reset to their ``nominal_defaults`` values, then the new state's setpoints are applied. Parameter updates are batched per target node and issued asynchronously; an update that fails makes the filter throw rather than being logged and ignored, so the robot does not keep driving on a value a zone was meant to change. Every transition publishes the new state id on ``state_event_topic``.

`<filter name>`: is the corresponding plugin name selected for this type.

`<state name>`: is a state name listed in ``states``.

`<setpoint name>`: is a setpoint name listed in the state's ``setpoints``.

:``<filter name>``.enabled:

  ====== =======
  Type   Default
  ------ -------
  bool   True
  ====== =======

  Description
    Whether it is enabled.

:``<filter name>``.filter_info_topic:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    Name of the incoming `CostmapFilterInfo <https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/msg/CostmapFilterInfo.msg>`_ topic having filter-related information. Published by Costmap Filter Info Server along with filter mask topic. For more details about Map and Costmap Filter Info servers configuration please refer to the :ref:`configuring_map_server` configuration page.

:``<filter name>``.transform_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.1
  ====== =======

  Description
    Time with which to post-date the transform that is published, to indicate that this transform is valid into the future. Used when filter mask and current costmap layer are in different frames.

:``<filter name>``.state_event_topic:

  ====== ===================
  Type   Default
  ------ -------------------
  string "zone_filter_state"
  ====== ===================

  Description
    Topic of ``std_msgs::msg::UInt8`` type to publish the new state id to on every state transition.

:``<filter name>``.states:

  ============== =======
  Type           Default
  -------------- -------
  vector<string> {}
  ============== =======

  Description
    Names of the declared states. Each name opens a ``<filter name>.<state name>`` parameter namespace holding the state's ``id`` and ``setpoints``. When empty, the filter warns and only handles state ``0`` (reset).

:``<filter name>``.<state name>.id:

  ==== =======
  Type Default
  ---- -------
  int  0
  ==== =======

  Description
    Filter mask cell value that selects this state. Valid range is ``[1, 255]``; ``0`` is reserved for the reset state. An out-of-range id logs an error and the state is skipped.

  Note
    An ``OccupancyGrid`` cell is ``int8``, so only values up to ``127`` can actually appear in a mask; negative cells are treated as unknown.

:``<filter name>``.<state name>.setpoints:

  ============== =======
  Type           Default
  -------------- -------
  vector<string> {}
  ============== =======

  Description
    Names of the setpoint entries for this state. Each name opens a ``<filter name>.<state name>.<setpoint name>`` parameter namespace holding ``node``, ``parameter`` and ``value``. A state with no valid setpoints logs a warning.

:``<filter name>``.<state name>.<setpoint name>.node:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    Name of the node whose parameter this setpoint sets. An empty ``node`` or ``parameter`` logs an error and the setpoint is skipped.

:``<filter name>``.<state name>.<setpoint name>.parameter:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    Full name of the parameter to set on the target node, e.g. ``FollowPath.max_vel_x``.

:``<filter name>``.<state name>.<setpoint name>.value:

  ======= =======
  Type    Default
  ------- -------
  dynamic N/A
  ======= =======

  Description
    Value to set. Declared with dynamic typing, so a setpoint can carry a parameter of any type (double, int, bool, string or list). An unset ``value`` logs an error and the setpoint is skipped.

  Note
    YAML typing applies to the target parameter: write ``0.0``, not ``0``, for a double-typed parameter.

:``<filter name>``.nominal_defaults:

  ============== =======
  Type           Default
  -------------- -------
  vector<string> {}
  ============== =======

  Description
    Names of the nominal default entries: the baseline values the state ``0`` reset restores. Each name opens a ``<filter name>.nominal_defaults.<name>`` parameter namespace holding ``node``, ``parameter`` and ``value``, the same shape as a setpoint. A state setpoint with no matching nominal default (same node and parameter) logs a warning at configuration load: the state ``0`` reset will not restore that parameter.

  Note
    ``nominal_defaults`` is both this list and the namespace its entries live under. In YAML, write the entry keys in dotted form (``nominal_defaults.fwd_speed:``) so the ``nominal_defaults`` key is not repeated, as in the example below.

Example
*******
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
            state_event_topic: "zone_filter_state"
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