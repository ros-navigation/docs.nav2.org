.. _graph_generation: 

Graph Generation
****************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

Overview
========
This tutorial walks a user through generating a graph for the nav2 router.

Requirements
============
Follow https://www.qgis.org/en/site/forusers/download.html to install QGIS.

Tutorial Steps
==============

1- Open QGIS and create a new project. Set the project coordinate reference system to `WGS 84/ Pseudo-Mercator`.

|

 .. image:: images/graph_generation/coordinate_reference_system.png
    :height: 400px
    :width: 600px
    :align: center

|

2- If you don't need to adjust your raster image to the map frame, click
`Layer -> Add Layer -> Add Raster Layer`. Select your raster image and click `Add`.

3- If you need to adjust your raster image to the map frame, open the georeferencer tool
by clicking `Raster -> Georefencer`. Set the trasformation settings to `Linear` for `Transformation 
type`, `WGS 84/ Pseudo-Mercator` for `Traget SRS` and set your desired path for the `Output Raster`. 

|

 .. image:: images/graph_generation/transformation_settings.png
    :height: 500px
    :width: 300px
    :align: center

|

Select the raster image you wish to georeference and place at least three georeferencer control points. Apply the
transformation.

|

 .. image:: images/graph_generation/georeferencer.png
    :height: 400px
    :width: 600px
    :align: center

|

Drag and drop the raster file output into the layers window.

|

 .. image:: images/graph_generation/raster_layer.png
    :height: 400px
    :width: 600px
    :align: center

|

4- Now that we have the raster layer in the correct coordinate system we can start placing nodes.


Select `Layer -> Create Layer -> New ShapeFile Layer`. Set the shapefile layer setting to be 
`nodes` for the `File name`, `points` for the `Geometry type` and `WGS 84/ Pseudo-Mercator` for the coordinate system. Press `OK`. 

|

 .. image:: images/graph_generation/node_layer.png
    :height: 600px
    :width: 400px
    :align: center

|

Click on the node layer and use the tool bar to start adding nodes. The IDs field can be left null. 

|

 .. image:: images/graph_generation/nodes.png
    :height: 600px
    :width: 400px
    :align: center

|


5- Now an edge layer can be created. Select `Layer -> Create Layer -> New ShapeFile Layer`. Set the shapefile layer setting to be 
`edges` for the `File name`, `LineString` for the `Geometry type` and `WGS 84/ Pseudo-Mercator` for the coordinate system. Press `OK`.

|

 .. image:: images/graph_generation/edge_layer.png
    :height: 400px
    :width: 600px
    :align: center

|

Click on the edge layer and use the tool bar to start adding edges. Use the magnet tool to snap
the start and end points of the edge to nodes. 

|

 .. image:: images/graph_generation/edges.png
    :height: 600px
    :width: 400px
    :align: center

|

6- Now we will add ids for all nodes and edges. 
Select the node layer and then click on the `Field calculator` tool. Check the `Update existing
field` box and select `id` in the dropdown menu. Add `@row_number` to the `expression field`` and click `Ok`.

|

 .. image:: images/graph_generation/field_calculator.png
    :height: 400px
    :width: 600px
    :align: center

|

This will generate ids for each node. To verify that the ids have been generated, right click on
the node layer and select `Open attribute table`. This will display the current attributes associated with the node layer. 

Follow the same process for the edges but swap the `@row_number` for `@row_number + <number_of_nodes>`. 


7- Now that we have our node and edge layers with IDs, we can associate node IDs with edge IDs. 

Select `Database -> DB manager`. Expand `Virtual layers` and expand `Project layers`. Open up
the SQL window. In the SQL window load in the association script by selecting `Load File`. 
Execute the script. Load the new layer by checking the `Load as new layer` box and clicking `Load`. 
This layer will be refered to as connected edges. 

|

 .. image:: images/graph_generation/sql_script.png
    :height: 400px
    :width: 600px
    :align: center

|

8- We are now ready to export the node and edge layer as geojson files. Right click on the nodes layer and select 
`Export -> Save Feature As`. Set the format to geojson and verify the coordinate reference system is set to `WGS 84/ Pseudo-Mercator`.

|

 .. image:: images/graph_generation/export_to_geojson.png
    :height: 300px
    :width: 400px
    :align: center

|

Follow the same process for the connected edges layer.

9- Create a new geojson file called graph. Copy the nodes geojson file into the graph file. Insert the features
from the connected edges geojson file into the features tag in the graph file.

|

 .. image:: images/graph_generation/geojson_graph.png
    :height: 600px
    :width: 400px
    :align: center

|

10- Congratulations! Your graph is ready to be consumed by the nav2 route! If you wish to add nodes or edges to your graph, 
edit the layers and repeat the proccess from step 6. If you wish to adjust the graph, make sure `Editing` is toggled on for the 
node and edge layer. Then select `Vertex Tool(all Layers)` and toggle on `Topological Editing`. Repeat the proccess from step 7. 


Happy Routing!

