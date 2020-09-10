.. _navigation2_with_keepout_filter:

(Costmap Filters) Navigating with Keepout Zones
***********************************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700" height="450" src="https://www.youtube.com/embed/p6wRsbgvgv0?autoplay=1" frameborder="1" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
      </div>
    </h1>

Overview
========

This tutorial shows how to simply utilize keep-out/safety zones where robots will never enter and preferred lanes for robots moving in industries and warehouses. All this functionality is being covered by ``KeepoutFilter`` costmap filter plugin which will be enabled and used in this document.

Requirements
============

It is assumed that ROS 2, Gazebo and TurtleBot3 packages are installed or built locally. Please make sure that Navigation2 project is also built locally as it was made in :ref:`build-instructions`.

Tutorial Steps
==============

1. Prepare map mask in the same way as any other PGM/PNG/BMP map made. For ``KeepoutFilter`` the passibility is proportional to color intesity (darker colors means more impassable areas). Black color means keep-out zone where robot will never enter or pass throuh. It should be noted that map itself and map mask might have different sizes, origin and resolution.

2. CostmapFilters are Costamp2D plugins. Add ``KeepoutFilter`` plugin in params of local and/or global costmap:

.. code-block:: text

  global_costmap:
    global_costmap:
      ros__parameters:
        ...
        plugins: ["static_layer", "obstacle_layer", "inflation_layer", "keepout_filter"]
        ...

Also, ``keepout_filter`` should have the following parameters defined:

- ``plugin``: type of plugin
- ``filter_info_topic``: filter info topic name

.. code-block:: text

        ...
        keepout_filter:
          plugin: "nav2_costmap_2d::KeepoutFilter"
          enabled: True
          filter_info_topic: "costmap_filter_info"

NOTE: Enabling ``KeepoutFilter`` for ``global_costmap`` only will cause path planner to build path plan bypassing keepout zones. Enabling ``KeepoutFilter`` for ``local_costmap`` only will cause robot won't go into keepout zones even if path planner makes the path through keepout zones. So, the best practive is to enable ``KeepoutFilter`` for both global and local costmaps.

3. Filter info topic should be published by Semantic Map Server. Until this server will be developed, dummy filter info publisher was made (placed in ``nav2_costmap_2d/test/costmap_filter_info/`` directory). Open its ``costmap_filter_info.launch.py`` file and ensure that proper ``namespace`` is set. Then launch dummy info publisher:

.. code-block:: text

  ros2 launch src/navigation2/nav2_costmap_2d/test/test_launch_files/costmap_filter_info.launch.py

It will launch dummy info publisher and map server set to mask publishing.

4. Then run navigation2 stack as written in :ref:`getting_started`:

.. code-block:: text

  ros2 launch nav2_bringup tb3_simulation_launch.py

And check that filter is working properly for global costmaps:

.. image:: images/Navigation2_with_Keepout_Filter/keepout_global.gif
    :width: 700px
    :align: center
    :alt: Animated gif with KeepoutFilter enabled for global costmaps

and local costmaps:

.. image:: images/Navigation2_with_Keepout_Filter/keepout_local.gif
    :width: 700px
    :align: center
    :alt: Animated gif with KeepoutFilter enabled for local costmaps
