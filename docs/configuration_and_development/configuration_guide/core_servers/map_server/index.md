# Map Server { #map-server-index }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_map_server).

The Map server package implements various components for handling grid maps, including loading, saving, and publishing maps and their metadata. Currently the following components are supported in Nav2:

<div class="grid cards bottom-align" markdown>

- :material-map: **Map Server**

    ---
    Loads and serves static occupancy grid maps.

    [:octicons-arrow-right-24: Go][map-server]

- :material-content-save: **Map Saver**

    ---
    Saves maps from SLAM to disk.

    [:octicons-arrow-right-24: Go][map-saver]

- :material-filter: **Costmap Filter Info Server**

    ---
    Publishes spatial filter metadata for keepout/speed zones.

    [:octicons-arrow-right-24: Go][costmap-filter-info-server]

- :material-vector-polygon: **Vector Object Server**

    ---
    Serves dynamic geometric map objects (polygons, lines, points).

    [:octicons-arrow-right-24: Go][vector-object-server]

</div>
