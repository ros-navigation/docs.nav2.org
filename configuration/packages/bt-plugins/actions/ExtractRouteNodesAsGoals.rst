.. _bt_extract_route_nodes_as_goals_action:

ExtractRouteNodesAsGoals
========================

Concatenates two paths into a single path, in order such that the output is ``input_path1 + input_path2``.
May be used with multiple of these calls sequentially to concatenate multiple paths.

Input Ports
-----------

:route:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  nav2_msgs/Route                 N/A
  =============================== =======

  Description
        Route to convert its ``nodes`` into Goals.

Output Ports
------------

:goals:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  nav_msgs/Goals                  N/A
  =============================== =======

  Description
        Goals comparing the route's ``nodes``.

Example
-------

.. code-block:: xml

    <ExtractRouteNodesAsGoals route="{route}" goals="{route_goals}"/>
