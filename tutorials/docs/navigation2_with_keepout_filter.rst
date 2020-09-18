.. _navigation2_with_keepout_filter:

(Costmap Filters) Navigating with Keepout Zones
***********************************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700" height="450" src="https://www.youtube.com/embed/jeMyOWH9HHA?autoplay=1" frameborder="1" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
      </div>
    </h1>

Overview
========

This tutorial shows how to simply utilize keep-out/safety zones where robots can't enter and preferred lanes for robots moving in industries and warehouses. All this functionality is being covered by ``KeepoutFilter`` costmap filter plugin which will be enabled and used in this document.

Requirements
============

It is assumed that ROS 2, Gazebo and TurtleBot3 packages are installed or built locally. Please make sure that Navigation2 project is also built locally as it was made in :ref:`build-instructions`.

Tutorial Steps
==============

1. Prepare map mask
-------------------

Map mask is the usual ROS map distributed through PGM, PNG or BMP raster file with its metadata YAML file. As an example you can use `keepout_mask.yaml <https://github.com/ros-planning/navigation2/tree/main/nav2_bringup/bringup/maps/keepout_mask.yaml>`_ and `keepout_mask.pgm <https://github.com/ros-planning/navigation2/tree/main/nav2_bringup/bringup/maps/keepout_mask.pgm>`_ files. Mask is also could be made manually by creating an image with PGM/PNG/BMP format and writing corresponding metadata in a YAML file. One of the easy ways how to create map mask - is to make it from world map. For this you need to copy your world map (say ``<map.pgm>``) to a new file (``<mask.pgm>``) and copy YAML metadata as well (``<map.yaml>`` -> to ``<mask.yaml>``). Then open ``<mask.pgm>`` in your favourite raster graphics editor. Each pixel there means an encoded information for the specific costmap filter you are going to use. For ``KeepoutFilter`` pixel color intesity is proportional to the passibility of area corresponting to this pixel: darker colors means more impassable areas. Black color covers keep-out zones where robot will never enter or pass throuh. After completing with the ``<mask.pgm>``, correct the ``image`` field in ``<mask.yaml>`` and new map mask is ready to use.

``KeepoutFilter`` also covers preferred lanes case, where robots should moving only on pre-defined lanes and permitted areas e.g. in warehouses. To use this feaure you need to prepare the mask map where lanes and permitted areas will be marked with "white" color while all other areas will be "black". TIP: Typically, amount of pixels belonging to lanes are much less than pixels covering other areas. In this case initially, all lanes data might be drawn as black-on-white and then (just before saving a file as PGM) "color inversion" tool might be used.

.. note::

  World map itself and map mask could have different sizes, origin and resolution which might be useful e.g. for cases when map mask is covering smaller areas on maps or when one map mask is used repeatedly many times (like annotating a keepout zone for same shape rooms in the hotel).

2. Enable Keepout Filter
------------------------

CostmapFilters are Costamp2D plugins. Enable ``KeepoutFilter`` plugin in Costmap2D. For that plugin name ``keepout_filter`` should be added to ``plugins`` parameter in ``nav2_params.yaml`` for ``global_costmap`` in order to be enabled in run-time for Planner Server. ``keepout_filter`` plugin should have the following parameters defined:

- ``plugin``: type of plugin. In our case ``nav2_costmap_2d::KeepoutFilter``.
- ``filter_info_topic``: filter info topic name, ``costmap_filter_info`` (see next chapter for more details).

For exaple, in ``nav2_params.yaml``:

.. code-block:: text

  global_costmap:
    global_costmap:
      ros__parameters:
        ...
        plugins: ["static_layer", "obstacle_layer", "inflation_layer", "keepout_filter"]
        ...
        keepout_filter:
          plugin: "nav2_costmap_2d::KeepoutFilter"
          enabled: True
          filter_info_topic: "costmap_filter_info"

To use this filter with a Controller, ``keepout_filter`` should be enabled by specifying ``local_costmap`` names instead of ``global_costmap`` in all steps from above.

It is important to note that enabling ``KeepoutFilter`` for ``global_costmap`` only will cause path planner to build path plan bypassing keepout zones. Enabling ``KeepoutFilter`` for ``local_costmap`` only will cause robot won't go into keepout zones even if path planner makes the path through keepout zones. So, the best practice is to enable ``KeepoutFilter`` for both global and local costmaps by adding it both in ``global_costmap`` and ``local_costmap`` in ``nav2_params.yaml``. Once enabled for ``global_costmap`` the plugin will operate in the same namespace, so for ``local_costmap`` the ``filter_info_topic`` parameter should be adjusted as follows:

.. code-block:: text

  global_costmap:
    global_costmap:
      ros__parameters:
        ...
        plugins: ["static_layer", "obstacle_layer", "inflation_layer", "keepout_filter"]
        ...
        keepout_filter:
          plugin: "nav2_costmap_2d::KeepoutFilter"
          enabled: True
          filter_info_topic: "costmap_filter_info"
  ...
  local_costmap:
    local_costmap:
      ros__parameters:
        ...
        plugins: ["voxel_layer", "inflation_layer", "keepout_filter"]
        ...
        keepout_filter:
          plugin: "nav2_costmap_2d::KeepoutFilter"
          enabled: True
          filter_info_topic: "/global_costmap/costmap_filter_info"

3. Configure Costmap Filter Info Publisher Server
-------------------------------------------------

According to the feature design, map mask is being published along with costmap filter info messages of ``nav2_msgs/msg/CostmapFilterInfo.msg`` type. These messages should be published by Semantic Map Server. Until this server will be developed, Costmap Filter Info Publisher Server was made (placed in ``nav2_map_server/src/costmap_filter_info/`` directory). Costmap Filter Info Publisher Server ROS parameters are listed in :ref:`configuring_map_server` page. This server is launching along with mask publishing Map Server from a ``costmap_filter_info.launch.py`` file which included into ``tb3_simulation_launch.py`` main launch-file. ``tb3_simulation_launch.py`` file has following parameters related costmap filters:

- ``costmap_filters``: Whether use Costmap Filters feature. ``False`` by default.
- ``filter_namespace``: Name of namespace where costmap filter was enabled: ``global_costmap``/``local_costmap`` in order to be enabled for Controller/Planner Server. By default it takes ``global_costmap`` value.
- ``mask``: Full path to map mask yaml file to load. By default it points to ``keepout_mask.yaml`` placed in ``nav2_bringup`` package.

Parameters used by Costmap Filter Info Publisher Server and mask Map Server are written in ``nav2_params.yaml`` file and belongs to ``global_costmap`` or ``local_costmap`` depending on in what namespace costmap filter was chosen. For ``KeepoutFilter`` enabled for ``global_costmap`` it may look as follows:

.. code-block:: text

  global_costmap:
    ...
    costmap_filter_info_server:
      ros__parameters:
        use_sim_time: true
        type: 0
        filter_info_topic: "costmap_filter_info"
        mask_topic: "map_mask"
        base: 0.0
        multiplier: 1.0
    map_mask_server:
      ros__parameters:
        use_sim_time: true
        frame_id: "map"
        topic_name: "map_mask"
        yaml_filename: "keepout_mask.yaml"

.. note::

  When ``KeepoutFilter`` is enabled for both global and local costmaps, there is no need to duplicate Costmap Filter Info Publisher and mask Map Server instances. Instead of this, leave ``costmap_filter_info_server`` and ``map_mask_server`` parameters to be belonging to ``global_costmap`` as shown in the example above. One thing is required to specify ``mask_topic`` parameter to be exactly equal to: ``/global_costmap/map_mask`` in order to avoid namespaces confusion.

4. Run Navigation2 stack
------------------------

After Keepout Filter, Costmap Filter Info Publisher Server and mask Map Server were configured, run navigation2 stack as written in :ref:`getting_started`, with ``costmap_filters`` parameter enabled for global costmaps and both global+local costmaps configured:

.. code-block:: bash

  ros2 launch nav2_bringup tb3_simulation_launch.py costmap_filters:=True

... and for local costmaps only configured:

.. code-block:: bash

  ros2 launch nav2_bringup tb3_simulation_launch.py costmap_filters:=True filter_namespace:=local_costmaps

And check that filter is working properly as in the pictures below (on left side keepout filter enabled for the global costmap, on right - for the local):

.. image:: images/Navigation2_with_Keepout_Filter/keepout_global.gif
    :width: 400px
    :align: left
    :alt: Animated gif with KeepoutFilter enabled for global costmaps

.. image:: images/Navigation2_with_Keepout_Filter/keepout_local.gif
    :width: 400px
    :align: right
    :alt: Animated gif with KeepoutFilter enabled for local costmaps
