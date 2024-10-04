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
  double         [20.0]
  ============== ========

  Description
    Expected planner frequency. If the current frequency is less than the expected frequency, display the warning message.

:action_server_result_timeout:

  ====== ======= ======= 
  Type   Default Unit
  ------ ------- -------
  double 10.0    seconds
  ====== ======= =======

  Description
    The timeout value (in seconds) for action servers to discard a goal handle if a result has not been produced. This used to default to
    15 minutes in rcl but was changed to 10 seconds in this `PR #1012 <https://github.com/ros2/rcl/pull/1012>`_, which may be less than
    some actions in Nav2 take to run. For most applications, this should not need to be adjusted as long as the actions within the server do not exceed this deadline. 
    This issue has been raised with OSRF to find another solution to avoid active goal timeouts for bookkeeping, so this is a semi-temporary workaround
    
:bond_heartbeat_period:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.1
  ============== =============================

  Description
    The lifecycle node bond mechanism publishing period (on the /bond topic). Disabled if inferior or equal to 0.0.

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
        planner_plugins: ['GridBased']
        GridBased:
          plugin: 'nav2_navfn_planner::NavfnPlanner' # In Iron and older versions, "/" was used instead of "::"
