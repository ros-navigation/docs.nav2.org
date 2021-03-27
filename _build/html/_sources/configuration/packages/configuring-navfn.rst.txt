.. _configuring_navfn:

NavFn Planner
#############

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_navfn_planner

The Navfn Planner plugin implements a wavefront Dijkstra or A* expanded planner.

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
    Whether to use A*. If false, uses Dijkstra's expansion.

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
        planner_plugins: ['GridBased']
        GridBased:
          plugin: 'nav2_navfn_planner/NavfnPlanner'
          use_astar: True
          allow_unknown: True
          tolerance: 1.0
