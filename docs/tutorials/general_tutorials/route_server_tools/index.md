# Route Server Tools { #route-server-tools }

Route Server tools are essential components in the Nav2 ecosystem that help users create, edit, and manage route graphs for robot navigation. A route graph is a representation of valid paths that a robot can follow in its environment, consisting of nodes (waypoints) and edges (connections between waypoints). These tools simplify the process of defining preferred paths and restricted areas for robot navigation.

Unlike free-space planning, route-based navigation ensures that robots follow specific, predefined paths, which is particularly useful in:

- Industrial environments where specific routes must be followed
- Warehouse operations requiring structured movement patterns
- Facilities with restricted areas or preferred paths
- Multi-robot coordination scenarios where predefined routes help prevent conflicts
- Large scale outdoor urban or natural environments

<figure markdown="span">
  ![](assets/route_homepage_demo.gif){ width="100%" }
  <figcaption>Demonstration of Nav2 route tool using the Turtlebot4 Gazebo simulation environment visualised through RViz.</figcaption>
</figure>

## Provided Tools

<div class="grid cards bottom-align" markdown>

- :material-map-marker-plus: **Using the Nav2 Route Tool**

    ---
    Interactive route creation in RViz.

    [:octicons-arrow-right-24: Go][using-the-nav2-route-tool]

- :material-pencil-ruler: **Route Graph Generation LIF Editor**

    ---
    Generate route graphs using the LIF editor tool.

    [:octicons-arrow-right-24: Go][route-graph-generation-lif-editor]

- :material-map-marker-path: **Route Graph Generation**

    ---
    Programmatic route graph creation.

    [:octicons-arrow-right-24: Go][route-graph-generation]

- :material-scatter-plot: **SWAGGER Route Graph Generation**

    ---
    Generate route graphs from SWAGGER/VDA5050 definitions.

    [:octicons-arrow-right-24: Go][swagger-route-graph-generation]

</div>

## Description

There are several tools available for creating and editing route graphs for the Nav2 Route Server:

1. **Nav2 Route Tool**: An Rviz panel that allows users to create, edit, and manage route graphs directly within the ROS 2 environment. It supports loading existing graphs, adding/editing/removing nodes and edges, and saving changes to files.
2. **VDA LIF Editor**: A web-based tool that allows users to create route graphs using floor plan images. It can generate both GeoJSON and LIF formats, making it particularly useful for standardized route creation. No installation is required as it runs in a web browser.
3. **Manual Route Graph Generation**: For those who prefer direct file editing, route graphs can be created manually in GeoJSON format using QGIS. This method provides the most control over the graph structure but requires understanding of the GeoJSON format.
4. **Swagger Generation**: A tool that automatically generates route graphs from raster images using GPU-aided geometric techniques for unstructured, semistructured, or applications where specific lanes and manual annotations are not required.

Choose the tool that best fits your needs based on your workflow and requirements.

## Demonstration

A demonstration of the assembled route server is provided to the Turtlebot4 Gazebo simulation for the Depot and Warehouse maps.

In order to run the demonstration, run the following command:
```shell
ros2 launch nav2_simple_commander route_example_launch.py
```

This will by default initialize the Turtlebot4 in the depot map in the **3rd node** and will navigate to the **13rd node**.

<figure markdown="span">
  ![](assets/route_depot_demo.gif){ width="90%" }
  <figcaption>Turtlebot4 moving from the 3rd node to the 13th node in the depot map.</figcaption>
</figure>

In order to run the demonstration in the warehouse map, you can set the `MAP_TYPE` variable in the
[route_example_launch.py](https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_simple_commander/launch/route_example_launch.py) file to `warehouse` and run the same command as above.

<figure markdown="span">
  ![](assets/route_warehouse_demo.gif){ width="90%" }
  <figcaption>Turtlebot4 moving from the 0th node to the 61th node in the warehouse map.</figcaption>
</figure>

For more information on how to configure the start and goal nodes, or the supported maps, please refer to the [Configuration Guide][route-server] page.
