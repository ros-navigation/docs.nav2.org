.. _route_graph_generation: 

Route Graph Generation
**********************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

Overview
========
This tutorial walks a user through generating a graph for the nav2 route server.

Requirements
============
Follow https://www.qgis.org/en/site/forusers/download.html to install QGIS.

Tutorial Steps
==============

1- Open QGIS and create a new project. Set the project coordinate reference system by selecting
`project->properties->CRS`. Set the coordinate system to `WGS 84/ Pseudo-Mercator` which will work well for
small and large scale maps.

|

 .. image:: images/route_graph_generation/coordinate_reference_system.png
    :height: 1097px
    :width: 1064px
    :align: center

|

2- If you don't need to adjust your raster image to the map frame, click
`Layer -> Add Layer -> Add Raster Layer`. Select your raster image and click `Add`.

3- If you need to adjust your raster image to the map frame, open the georeferencer tool
by clicking `Raster -> Georefencer`. Set the trasformation settings to `Linear` for `Transformation 
type`, `WGS 84/ Pseudo-Mercator` for `Traget SRS` and set your desired path for the `Output Raster`. 

|

 .. image:: images/route_graph_generation/transformation_settings.png
    :height: 757px
    :width: 458px
    :align: center

|

Select the raster image you wish to georeference and place at least three georeferencer control points. Apply the
transformation.

|

 .. image:: images/route_graph_generation/georeferencer.png
    :height: 807px
    :width: 1460px
    :align: center

|

Drag and drop the raster file output into the layers window.

|

 .. image:: images/route_graph_generation/raster_layer.png
    :height: 702px
    :width: 1051px
    :align: center

|

4- Now that we have the raster layer in the correct coordinate system we can start placing nodes.


Select `Layer -> Create Layer -> New ShapeFile Layer`. Set the shapefile layer setting to be 
`nodes` for the `File name`, `points` for the `Geometry type` and `WGS 84/ Pseudo-Mercator` for the coordinate system. Press `OK`. 

|

 .. image:: images/route_graph_generation/node_layer.png
    :height: 1041px
    :width: 887px
    :align: center

|

In order to have the `id` field auto increment, right click on the layer and select the `Attribute Form`. 
Expand the `Fields` drop down menu and select `id`. Select the `Expression Dialog` that is next to the `Default value`. 

|

 .. image:: images/route_graph_generation/attribute_form.png
    :height: 788px
    :width: 1107px
    :align: center

|


Within the `Expression Dialog` select `Import user expressions` and import `increment_node_id.json`. Expand `User expressions` and double click on `increment_node_id`. 
The expression should show up in the left window. Click `OK` to save the expression and exit the `Expression Dialog`. Then click `Apply` and `OK` and save and exit the `Attributes Form`.
This will increment the node `id` by one every time a new node is added. The first node `id` will be zero. 

|

 .. image:: images/route_graph_generation/expression_dialog.png
    :height: 776px
    :width: 950px
    :align: center

|


Click on the node layer and then on the pencil icon to start editing the layer. To add points, click on `Add points feature`. Start adding nodes by clicking in the main window. 

|

 .. image:: images/route_graph_generation/nodes.png
    :height: 1922px
    :width: 1082px
    :align: center

|


5- Now an edge layer can be created. Select `Layer -> Create Layer -> New ShapeFile Layer`. Set the shapefile layer setting to be 
`edges` for the `File name`, `LineString` for the `Geometry type` and `WGS 84/ Pseudo-Mercator` for the coordinate system. Press `OK`.
To auto incrment the `id` field follow the same steps as above except replace `increment_node_id.json` with `increment_edge_id.json`. 
This will increment the edge `id` by one every time a new edge is added. The first edge `id` will be `10000`. 

|

 .. image:: images/route_graph_generation/edge_layer.png
    :height: 1041px
    :width: 887px
    :align: center

|

Click on the edge layer and then on the pencil icon to start editing the layer. To add edges, click on `Add Line feature`. Start addig edges by clicking twice in the main window. 
(First point is start, second point is end). Press `Esc` when you have finished adding the two points.  

|

 .. image:: images/route_graph_generation/edges.png
    :height: 1922px
    :width: 1082px
    :align: center

|


7- Now that we have our node and edge layers, we can associate node IDs with edge IDs. 

Select `Database -> DB manager`. Expand `Virtual layers` and expand `Project layers`. Open up
the SQL window by clicking on the script icon in the top left corner. In the SQL window load in the association script by selecting `Load File`. 
Execute the script. Load the new layer by checking the `Load as new layer` box and clicking `Load`. The script associates the start and end of a line by matching the node 
point to the line string end or start point. We will refer to this new layer as `connected_edges`. 

|

 .. image:: images/route_graph_generation/db_manager.png
    :height: 700px
    :width: 1060px
    :align: center

|

8- We are now ready to export the node and edge layer as geojson files. Execute `export_shapefiles.py <prefix_of_file> <path_to_connected_edges_shapefile> <path_to_nodes_shapefile>`. 
This script converts the nodes and edges shape file into a geojson file. 

9- Congratulations! Your graph is ready to be consumed by the nav2 route! If you wish to add nodes or edges to your graph, 
edit the layers and repeat the proccess from step 6. If you wish to adjust the graph, make sure `Editing` is toggled on for the 
node and edge layer. Then select `Vertex Tool(all Layers)` and toggle on `Topological Editing`. Repeat the proccess from step 7. 


Happy Routing!

