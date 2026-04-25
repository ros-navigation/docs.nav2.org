.. _zone_parameter_filter:

Zone Parameter Filter Parameters
================================

Zone Parameter Filter - is a Costmap Filter that applies a configurable
set of nav2 parameter overrides to one or more target nodes based on the
mask value at the robot's current pose. The filter is the mechanism; the
use-case lives in YAML — winter roads, school zones, indoor/outdoor
transitions, and construction zones all reduce to the same plugin with
different state maps.

The mask value is interpreted as a state ID. State ``0`` is reserved as
the special "reset" state, which restores all overridden parameters to
their YAML-declared nominal values. Each non-zero mask value (effective
range bounded by the ``int8_t`` ``OccupancyGrid`` transport, ``-128`` to
``127``; negative values are reserved for ``OCC_GRID_UNKNOWN`` and are
ignored) maps to a list of parameter overrides via the layer's
configuration.

Parameter sets are issued asynchronously via ``AsyncParametersClient``;
the filter never calls ``wait_for_service`` in the hot path. If a target
node's parameter service is not ready, the filter logs a throttled warning
and skips that target — the costmap update loop is not blocked.

`<filter name>`: is the corresponding plugin name selected for this type.

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

:``<filter name>``.target_nodes:

  ============== =======
  Type           Default
  -------------- -------
  vector<string> []
  ============== =======

  Description
    Required. Explicit list of target node names the filter is allowed to mutate. The state-override parser does longest-prefix-match against this list, so nested-namespace targets parse unambiguously. Catches typos at config-load. Example: ``["controller_server", "local_costmap"]``.

:``<filter name>``.state_ids:

  ============= =======
  Type          Default
  ------------- -------
  vector<int>   []
  ============= =======

  Description
    Required. List of mask values (each in ``[1, 127]``) that map to a configured state. Each listed value must have a corresponding ``state_<N>`` block. State ``0`` is the implicit reset state and is not listed here.

:``<filter name>``.state_<N>.<target_node>.<parameter_path>:

  ====== =======
  Type   Default
  ------ -------
  any    N/A
  ====== =======

  Description
    For each ``N`` in ``state_ids``, declare the parameter overrides to apply when the robot enters mask state ``N``. ``<target_node>`` must be one of the names listed in ``target_nodes``. ``<parameter_path>`` is the parameter name on that target node, dots and all (e.g., ``inflation_layer.inflation_radius``). The parameter type must match the target's declared type. Example: ``state_1.controller_server.FollowPath.max_vel_x: 0.5``.

:``<filter name>``.nominal_defaults.<target_node>.<parameter_path>:

  ====== =======
  Type   Default
  ------ -------
  any    N/A
  ====== =======

  Description
    Optional but strongly recommended. The value to restore for each parameter when the robot enters state ``0`` (reset). One entry per parameter that any state ``N`` overrides. Declarative rather than auto-captured because the underlying ``get_parameters`` and ``set_parameters`` services are separate ``services::Client`` instances on the target node — server-side ordering between a "capture-then-override" sequence is not guaranteed, so a late get response could capture the overridden value rather than the nominal. The filter logs a warning at config-load time for any state-N override that has no matching ``nominal_defaults`` entry; such parameters will not be restored on state-0 reset.

:``<filter name>``.state_event_topic:

  ====== =======
  Type   Default
  ------ -------
  string ""
  ====== =======

  Description
    Optional. If set to a non-empty topic name, the filter publishes a ``std_msgs::msg::UInt8`` message containing the new state ID on every state transition (including transitions to state ``0``). If left empty, no publisher is created.

:``<filter name>``.on_unknown_state:

  ====== =======
  Type   Default
  ------ -------
  string "warn"
  ====== =======

  Description
    Behavior when the mask value at the robot's pose is non-negative but not present in the configured ``state_ids`` map. Either ``"warn"`` (log a throttled warning, leave the current state unchanged) or ``"throw"`` (raise an exception, terminating the filter).

:``<filter name>``.on_param_set_failure:

  ====== =======
  Type   Default
  ------ -------
  string "warn"
  ====== =======

  Description
    Behavior when a target node's ``set_parameters`` call returns ``successful = false``. Either ``"warn"`` (log the failure and continue — the costmap update loop is preserved) or ``"throw"`` (raise an exception, terminating the filter). The default is ``"warn"`` because a hot-path exception in a costmap filter takes the entire navigation stack down; ``"throw"`` is offered for stacks that prefer hard correctness over operational continuity.

Example Fully-Described YAML
----------------------------

.. code-block:: yaml

    local_costmap:
      ros__parameters:
        plugins: ["voxel_layer", "inflation_layer"]
        filters: ["zone_parameter_filter"]
        zone_parameter_filter:
          plugin: "nav2_costmap_2d::ZoneParameterFilter"
          enabled: true
          filter_info_topic: "/zone_filter_info"
          state_event_topic: "/zone_filter_state"   # optional
          on_unknown_state: "warn"                   # or "throw"
          on_param_set_failure: "warn"               # or "throw"
          target_nodes: [controller_server, local_costmap]
          state_ids: [1, 2]
          state_1:                                   # winter / icy zone
            controller_server:
              FollowPath.max_vel_x: 0.5
            local_costmap:
              inflation_layer.inflation_radius: 0.8
          state_2:                                   # construction zone
            controller_server:
              FollowPath.max_vel_x: 0.2
            local_costmap:
              inflation_layer.inflation_radius: 1.2
          nominal_defaults:                          # state-0 restores these
            controller_server:
              FollowPath.max_vel_x: 1.0
            local_costmap:
              inflation_layer.inflation_radius: 0.55

Common Pitfalls
---------------

These are the operational sharp edges that have surfaced during design and review:

- **Target-node typos go silent at config-load until first transition.**
  An override under a ``state_<N>`` namespace whose first segment doesn't match an entry in ``target_nodes`` is rejected with a warning. The filter still becomes active and continues with the remaining valid overrides — but the typo'd parameter is never set. Always check the warn log on first activation.

- **State-to-state transitions only apply the new state's overrides.**
  If state 1 sets ``A`` and ``B`` and state 2 sets only ``A``, then a 1→2 transition writes the new value of ``A`` and leaves ``B`` at state 1's value. Only the special state ``0`` reset restores anything to ``nominal_defaults``. Plan ``nominal_defaults`` so any param touched by any state has a documented baseline.

- **The hot path is non-blocking by design.**
  If a target node's parameter service is not ready when ``process()`` runs (target node restarting, network blip, etc.), the override for that node is skipped that cycle and a throttled warning is logged. The filter does *not* call ``wait_for_service`` because that would stall the entire costmap update loop. Subsequent ``process()`` calls retry naturally.

- **Effective state-ID range is [1, 127], not [1, 255].**
  Although the documentation shows the conceptual range as 0–255, the underlying ``OccupancyGrid::data`` field is ``int8_t``, so values 128–255 arrive as their signed-negative complement. Negative mask values are reserved for ``OCC_GRID_UNKNOWN`` and are silently ignored at the robot's pose. If you need the full 0–255 range, wrap your mask publisher to reinterpret-cast unsigned to signed before publishing — but [1, 127] covers the vast majority of zone schemes.

- **Longest-prefix matching applies to ``target_nodes`` ordering.**
  When ``target_nodes`` contains both a node name and a namespace-prefixed sibling (e.g., ``["a", "a.b"]``), an override under ``state_<N>.a.b.foo`` is routed to ``a.b`` (longer match), not to ``a`` (with parameter path ``b.foo``). The filter sorts ``target_nodes`` by length descending at config-load to make this deterministic.
