.. _route_server_tools:

Route Server Tools
##################

Route Server tools are essential components in the Nav2 ecosystem that help users create, edit, and manage route graphs for robot navigation. A route graph is a representation of valid paths that a robot can follow in its environment, consisting of nodes (waypoints) and edges (connections between waypoints). These tools simplify the process of defining preferred paths and restricted areas for robot navigation.

Unlike free-space planning, route-based navigation ensures that robots follow specific, predefined paths, which is particularly useful in:

- Industrial environments where specific routes must be followed
- Warehouse operations requiring structured movement patterns
- Facilities with restricted areas or preferred paths
- Multi-robot coordination scenarios where predefined routes help prevent conflicts
- Large scale outdoor urban or natural environments

.. image:: ../docs/images/Navigation2_route_tool/route_homepage_demo.gif
    :width: 100%
    :align: center

.. centered:: *Demonstration of Nav2 route tool using the Turtlebot4 Gazebo simulation environment visualised through RViz.*

Provided Tools
**************

.. toctree::
  :maxdepth: 1

  route_server_tools/navigation2_route_tool.rst
  route_server_tools/route_graph_generation_lif_editor.rst
  route_server_tools/route_graph_generation.rst
  route_server_tools/swagger_generation.rst

Description
***********

There are several tools available for creating and editing route graphs for the Nav2 Route Server:

1. **Nav2 Route Tool**: An Rviz panel that allows users to create, edit, and manage route graphs directly within the ROS 2 environment. It supports loading existing graphs, adding/editing/removing nodes and edges, and saving changes to files.

2. **VDA LIF Editor**: A web-based tool that allows users to create route graphs using floor plan images. It can generate both GeoJSON and LIF formats, making it particularly useful for standardized route creation. No installation is required as it runs in a web browser.

3. **Manual Route Graph Generation**: For those who prefer direct file editing, route graphs can be created manually in GeoJSON format using QGIS. This method provides the most control over the graph structure but requires understanding of the GeoJSON format.

4. **Swagger Generation**: A tool that automatically generates route graphs from raster images using GPU-aided geometric techniques for unstructured, semistructured, or applications where specific lanes and manual annotations are not required.

Choose the tool that best fits your needs based on your workflow and requirements.

Demonstration
*************

| A demonstration of the assembled route server is provided to the Turtlebot4 Gazebo simulation for the Depot and Warehouse maps.
| In order to run the demonstration, run the following command:

.. code-block:: console

    ros2 launch nav2_simple_commander route_example_launch.py

This will by default initialize the Turtlebot4 in the depot map in the **3rd node** and will navigate to the **13rd node**.

.. image:: ../docs/images/Navigation2_route_tool/route_depot_demo.gif
    :width: 90%
    :align: center

.. centered:: *Turtlebot4 moving from the 3rd node to the 13th node in the depot map.*

In order to run the demonstration in the warehouse map, you can set the ``MAP_TYPE`` variable in the
`route_example_launch.py <https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/launch/route_example_launch.py>`_ file to ``warehouse`` and run the same command as above.

.. image:: ../docs/images/Navigation2_route_tool/route_warehouse_demo.gif
    :width: 90%
    :align: center

.. centered:: *Turtlebot4 moving from the 0th node to the 61th node in the warehouse map.*

For more information on how to configure the start and goal nodes, or the supported maps, please refer to the `Configuration Guide <https://docs.nav2.org/configuration/packages/configuring-route-server.html>`_ page.
