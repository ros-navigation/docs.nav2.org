.. _configuring_planner_server:

Planner Server
##############

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_planner

The Planner Server implements the server for handling the planner requests for the stack and host a map of plugin implementations.
It will take in a goal and a planner plugin name to use and call the appropriate plugin to compute a path to the goal.

Parameters
**********

:planner_plugin_ids:

  ============== ==============
  Type           Default                                               
  -------------- --------------
  vector<string> ['GridBased']            
  ============== ==============

  Description
    List of Mapped plugin names for parameters and processing requests.

:planner_plugin_types:

  ============== ===================================
  Type           Default                                               
  -------------- -----------------------------------
  vector<string> ['nav2_navfn_planner/NavfnPlanner']            
  ============== ===================================

  Description
    List of registered pluginlib planner types to load.

:expected_planner_frequency:

  ============== ========
  Type           Default
  -------------- --------
  double         [20.0]
  ============== ========

  Description
    Expected planner frequency. If the current frequency is less than the expected frequency, display the warning message.

Example
*******
.. code-block:: yaml

    planner_server:
      ros__parameters:
        planner_plugin_ids: ['GridBased']
        planner_plugin_types: ['nav2_navfn_planner/NavfnPlanner']
        expected_planner_frequency: 20.0
