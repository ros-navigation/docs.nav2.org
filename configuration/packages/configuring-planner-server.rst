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
  double         0.1
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
:service_introspection_mode:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "disabled"
  ============== =============================

  Description
    The introspection mode for services. Options are "disabled", "metadata", "contents".

Default Plugins
***************

When the :code:`planner_plugins` parameter is not overridden, the following default plugins are loaded:

  ================= =====================================================
  Namespace         Plugin
  ----------------- -----------------------------------------------------
  "GridBased"       "nav2_navfn_planner::NavfnPlanner"
  ================= =====================================================

Example
*******
.. code-block:: yaml

    planner_server:
      ros__parameters:
        expected_planner_frequency: 20.0
        costmap_update_timeout: 1.0
        service_introspection_mode: "disabled"
        planner_plugins: ['GridBased']
        GridBased:
          plugin: 'nav2_navfn_planner::NavfnPlanner' # In Iron and older versions, "/" was used instead of "::"
