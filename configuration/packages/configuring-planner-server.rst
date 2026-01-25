.. _configuring_planner_server:

Planner Server
##############

Source code on Github_.

.. _Github: https://github.com/ros-navigation/navigation2/tree/main/nav2_planner

The Planner Server implements the server for handling the planner requests for the stack and host a map of plugin implementations.
It will take in a goal and a planner plugin name to use and call the appropriate plugin to compute a path to the goal.
It also hosts the global costmap.

Parameters
**********

.. tabs::

  .. group-tab:: Lyrical and newer

    :planner_plugins:

      ============== ==============
      Type           Default
      -------------- --------------
      vector<string> ['grid_based']
      ============== ==============

      Description
        List of Mapped plugin names for parameters and processing requests.

      Note
        Each plugin namespace defined in this list needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:

        .. code-block:: yaml

            planner_server:
              ros__parameters:
                planner_plugins: ["grid_based"]
                grid_based:
                  plugin: "nav2_navfn_planner::NavfnPlanner" # In Iron and older versions, "/" was used instead of "::"
        ..

    :allow_partial_planning:

      ============== ========
      Type           Default
      -------------- --------
      bool           false
      ============== ========

      Description
        Allows planner server to output partial paths in the presence of obstacles when planning through poses. Otherwise planner fails and aborts the plan request in such a case by default.

    :expected_planner_frequency:

      ============== ========
      Type           Default
      -------------- --------
      double         20.0
      ============== ========

      Description
        Expected planner frequency. If the current frequency is less than the expected frequency, display the warning message.

    :bond_heartbeat_period:

      ============== =============================
      Type           Default
      -------------- -----------------------------
      double         0.25
      ============== =============================

      Description
        The lifecycle node bond mechanism publishing period (on the /bond topic). Disabled if inferior or equal to 0.0.

    :costmap_update_timeout:

      ============== ========
      Type           Default
      -------------- --------
      double         1.0
      ============== ========

      Description
        The timeout value (seconds) for the costmap to be fully updated before a planning request.

    :introspection_mode:

      ============== =============================
      Type           Default
      -------------- -----------------------------
      string         "disabled"
      ============== =============================

      Description
        The introspection mode for services and actions. Options are "disabled", "metadata", "contents".

    :allow_parameter_qos_overrides:

      ============== =============================
      Type           Default
      -------------- -----------------------------
      bool           true
      ============== =============================

      Description
        Whether to allow QoS profiles to be overwritten with parameterized values.

  .. group-tab:: Kilted and older

    :planner_plugins:

      ============== ==============
      Type           Default
      -------------- --------------
      vector<string> ['GridBased']
      ============== ==============

      Description
        List of Mapped plugin names for parameters and processing requests.

      Note
        Each plugin namespace defined in this list needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:

        .. code-block:: yaml

            planner_server:
              ros__parameters:
                planner_plugins: ["GridBased"]
                GridBased:
                  plugin: "nav2_navfn_planner::NavfnPlanner" # In Iron and older versions, "/" was used instead of "::"
        ..

    :allow_partial_planning:

      ============== ========
      Type           Default
      -------------- --------
      bool           false
      ============== ========

      Description
        Allows planner server to output partial paths in the presence of obstacles when planning through poses. Otherwise planner fails and aborts the plan request in such a case by default.

    :expected_planner_frequency:

      ============== ========
      Type           Default
      -------------- --------
      double         20.0
      ============== ========

      Description
        Expected planner frequency. If the current frequency is less than the expected frequency, display the warning message.

    :bond_heartbeat_period:

      ============== =============================
      Type           Default
      -------------- -----------------------------
      double         0.25
      ============== =============================

      Description
        The lifecycle node bond mechanism publishing period (on the /bond topic). Disabled if inferior or equal to 0.0.

    :costmap_update_timeout:

      ============== ========
      Type           Default
      -------------- --------
      double         1.0
      ============== ========

      Description
        The timeout value (seconds) for the costmap to be fully updated before a planning request.

    :introspection_mode:

      ============== =============================
      Type           Default
      -------------- -----------------------------
      string         "disabled"
      ============== =============================

      Description
        The introspection mode for services and actions. Options are "disabled", "metadata", "contents".

    :allow_parameter_qos_overrides:

      ============== =============================
      Type           Default
      -------------- -----------------------------
      bool           true
      ============== =============================

      Description
        Whether to allow QoS profiles to be overwritten with parameterized values.

Default Plugins
***************

When the :code:`planner_plugins` parameter is not overridden, the following default plugins are loaded:

.. tabs::

  .. group-tab:: Lyrical and newer

    ================= =====================================================
    Namespace         Plugin
    ----------------- -----------------------------------------------------
    "grid_based"       "nav2_navfn_planner::NavfnPlanner"
    ================= =====================================================

  .. group-tab:: Kilted and older

    ================= =====================================================
    Namespace         Plugin
    ----------------- -----------------------------------------------------
    "GridBased"       "nav2_navfn_planner::NavfnPlanner"
    ================= =====================================================

Example
*******

.. tabs::

  .. group-tab:: Lyrical and newer

    .. code-block:: yaml

      planner_server:
        ros__parameters:
          allow_partial_planning: false
          expected_planner_frequency: 20.0
          costmap_update_timeout: 1.0
          introspection_mode: "disabled"
          planner_plugins: ['grid_based']
          grid_based:
            plugin: 'nav2_navfn_planner::NavfnPlanner' # In Iron and older versions, "/" was used instead of "::"

  .. group-tab:: Kilted and older

    .. code-block:: yaml

      planner_server:
        ros__parameters:
          allow_partial_planning: false
          expected_planner_frequency: 20.0
          costmap_update_timeout: 1.0
          introspection_mode: "disabled"
          planner_plugins: ['GridBased']
          GridBased:
            plugin: 'nav2_navfn_planner::NavfnPlanner' # In Iron and older versions, "/" was used instead of "::"
