.. _filtering_of_noise-induced_obstacles:

Filtering of noise-induced obstacles
***********************************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_
- `How it works`_

.. image:: images/Filtering_of_noise-induced_obstacles/title.png
    :width: 1000px

Overview
========

This tutorial shows how to configure filtering of false obstacles caused by noise. This function is provided by the ``DenoiseLayer`` costmap layer plugin which will be enabled and used in this document.

Requirements
============

It is assumed that ROS 2, Gazebo and TurtleBot3 packages are installed or built locally. Please make sure that Nav2 project is also built locally as it was made in :ref:`build-instructions`.

Tutorial Steps
==============

1. Enable Denoise Layer
------------------------

Denoise Layer are Costamp2D plugin. You can enable the ``DenoiseLayer`` plugin in Costmap2D by adding ``denoise_layer`` to the ``plugins`` parameter in ``nav2_params.yaml``. You can place it in the ``global_costmap`` and (or) ``local_costmap`` to filter noise on a global or local map. The DenoiseLayer plugin should have the following parameter defined:

- ``plugin``: type of plugin. In our case ``nav2_costmap_2d::DenoiseLayer``.

Full list of parameters supported by ``DenoiseLayer`` are listed at :ref:`denoise` page.

It is important to note that ``DenoiseLayer`` typically should be placed before inflation_layer.
This is required to prevent inflation from noise-induced obstacles.
Moreover, ``DenoiseLayer`` expects a binary costmap with values ``0`` (free cell) or ``255`` (obstacle cell).
Values less than 255 will be interpreted as free cells when processed (but will not be distorted in the cost map).

To enable ``DenoiseLayer`` for both global and local costmaps, use the following configuration:

.. code-block:: text

  global_costmap:
    global_costmap:
      ros__parameters:
        ...
        plugins: ["static_layer", "obstacle_layer", "denoise_layer", "inflation_layer"]
        ...
        denoise_layer:
          plugin: "nav2_costmap_2d::DenoiseLayer"
          enabled: True
  ...
  local_costmap:
    local_costmap:
      ros__parameters:
        ...
        plugins: ["voxel_layer", "denoise_layer", inflation_layer"]
        ...
        keepout_filter:
          plugin: "nav2_costmap_2d::DenoiseLayer"
          enabled: True

.. note::

  The key to success in filtering noise is to understand its type and choose the right ``DenoiseLayer`` parameters.
  The default parameters are focused on fast removal of standalone obstacles.
  More formally, an obstacle is discarded if there are no obstacles among the adjacent eight cells.
  This should be sufficient in typical cases.

  If (1) a some sensor generates intercorrellated noise-induced obstacles and (2) small obstacles in the world are unlikely, small groups of obstacles can be removed.
  To configure the ``DenoiseLayer`` to such cases and understand how it works, refer to the section `How it works`_.

2. Run Nav2 stack
-----------------

After Denoise Layer was enabled for global/local costmaps, run Nav2 stack as written in :ref:`getting_started`:

.. code-block:: bash

  ros2 launch nav2_bringup tb3_simulation_launch.py

And check that filter is working properly: with the default parameters,
no standalone obstacles should remain on the cost map. This can be checked, for example, in RVis GUI.


How it works
==============

The plugin is based on two algorithms.

When parameter ``minimal_group_size`` = 2, the first algorithm turns on.
It apply erosion function with kernel from image below (left if ``group_connectivity_type`` = 4 or right if ``group_connectivity_type`` = 8) to the costmap.
As a result of this window function an image is created. The pixel of this image is 255 if there is an obstacle nearby, 0 in other case.
After that, obstacles corresponding to zero pixels are removed.

.. image:: images/Filtering_of_noise-induced_obstacles/3x3_kernels.png
    :width: 222px

This process is illustrated in the animation below (``group_connectivity_type`` = 4).
Obstacles marked at the end of the animation will be removed.

.. image:: images/Filtering_of_noise-induced_obstacles/dilate.gif
    :width: 600px

When parameter ``minimal_group_size`` > 2, the second algorithm is executed.
This is a generalized solution that allows you to remove groups of adjacent obstacles if their total number is less than ``minimal_group_size``.
To select groups of adjacent obstacles, the algorithm performs their segmentation.
The type of cell connectivity in one segment is determined by the parameter ``group_connectivity_type``.
Next, the size of each segment is calculated.
Obstacles segments with size less than the ``minimal_group_size`` are replaced with empty cells.
This algorithm is about 10 times slower, so use it with caution and only when necessary.
Its execution time depends on the size of the processed map fragment (and not depend on the value of ``minimal_group_size``).

This algorithm is illustrated in the animation below (``group_connectivity_type`` = 8).
Obstacles marked at the end of the animation will be removed (groups that size less 3).

.. image:: images/Filtering_of_noise-induced_obstacles/connected_components.gif
    :width: 600px

