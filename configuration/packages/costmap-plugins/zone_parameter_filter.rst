.. _zone_parameter_filter:

Zone Parameter Filter Parameters
================================

Zone Parameter Filter applies a configurable set of nav2 parameter
overrides to one or more target nodes based on the mask value at the
robot's current pose. Configure the per-state overrides in YAML.

The mask value is interpreted as a state ID. State ``0`` is the reset
state and restores parameters to their YAML-declared nominal values.
Mask values in ``[1, 127]`` map to configured states.

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
    Required. Explicit list of target node names the filter is allowed to mutate. Catches typos at config-load. Example: ``["controller_server", "local_costmap"]``.

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
    Value to restore for each parameter on state ``0`` (reset). Declared rather than auto-captured because ``get_parameters`` and ``set_parameters`` use separate underlying ``services::Client`` instances on the target node, so a "capture-then-override" sequence cannot guarantee FIFO ordering at the server. The filter logs a warning at config-load for any state-N override without a matching ``nominal_defaults`` entry; such parameters will not be restored on state-0 reset.

:``<filter name>``.state_event_topic:

  ====== ====================
  Type   Default
  ------ --------------------
  string "zone_filter_state"
  ====== ====================

  Description
    Topic name on which the filter publishes a ``std_msgs::msg::UInt8`` message containing the new state ID on every state transition (including transitions to state ``0``). The publisher is always created.

:``<filter name>``.on_param_set_failure:

  ====== =======
  Type   Default
  ------ -------
  string "throw"
  ====== =======

  Description
    Behavior when a target node's ``set_parameters`` call returns ``successful = false``. Either ``"throw"`` (raise an exception, terminating the filter) or ``"warn"`` (log the failure and continue — the costmap update loop is preserved). The default is ``"throw"`` for fail-safe behavior on persistent set-parameter faults; ``"warn"`` is offered for environments where transient RPC faults (target node mid-restart, parameter-service overloaded) are common and continuing through them is preferred over the lifecycle-stopping exception.

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
          state_event_topic: "zone_filter_state"     # default
          on_param_set_failure: "throw"              # default; or "warn"
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

Notes
-----

- **State-to-state transitions only apply the new state's overrides.**
  If state 1 sets ``A`` and ``B`` and state 2 sets only ``A``, then a 1→2 transition writes the new value of ``A`` and leaves ``B`` at state 1's value. Only the special state ``0`` reset restores anything to ``nominal_defaults``. Plan ``nominal_defaults`` so any param touched by any state has a documented baseline.
