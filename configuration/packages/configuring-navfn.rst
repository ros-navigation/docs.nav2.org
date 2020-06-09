.. _configuring_navfn:

NavFn Planner
#############

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_navfn_planner

The Navfn Planner plugin implements a wavefront Dij. or A* expanded planner.

``<name>`` is the corresponding planner plugin ID selected for this type.

Parameters
**********

:``<name>``.tolerance:

  ============== =======
  Type           Default
  -------------- -------
  double         0.5  
  ============== =======

  Description
    Tolerance in meters between requested goal pose and end of path.

:``<name>``.use_astar:

  ==== =======
  Type Default                                                   
  ---- -------
  bool False            
  ==== =======

  Description
    Whether to use A*, if false, uses Dijstra's expansion

:``<name>``.allow_unknown:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Whether to allow planning in unknown space.

Example
*******
.. code-block:: yaml

    planner_server:
      ros__parameters:
        planner_plugin_ids: ['GridBased']
        planner_plugin_types: ['nav2_navfn_planner/NavfnPlanner']
        GridBased.use_astar: True
        GridBased.allow_unknown: True
        GridBased.tolerance: 1.0
