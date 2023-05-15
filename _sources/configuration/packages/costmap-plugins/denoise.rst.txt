.. _denoise:

Denoise Layer Parameters
========================

``<denoise layer>`` is the corresponding plugin name selected for this type.

:``<denoise layer>``.enabled:

  ==== =======
  Type Default
  ---- -------
  bool True
  ==== =======

  Description
    Whether it is enabled.

:``<denoise layer>``.minimal_group_size:

  ==== =======
  Type Default
  ---- -------
  int  2
  ==== =======

  Description
    The minimum number of adjacent obstacles that should not be discarded as noise.

    | If 1 or less, all obstacles will be kept.
    | If 2, standalone obstacles (without neighbors in adjacent cells) will be removed.
    | If N, obstacles groups smaller than N will be removed.

:``<denoise layer>``.group_connectivity_type:

  ====== =======
  Type   Default
  ------ -------
  int    8
  ====== =======

  Description
    Obstacles connectivity type (is the way in which obstacles relate to their neighbors).
    Must be 4 or 8.

    | 4 - adjacent obstacles are connected horizontally and vertically.
    | 8 - adjacent obstacles are connected horizontally, vertically and diagonally.

Example
*******
.. code-block:: yaml

    local_costmap:
      local_costmap:
        ros__parameters:
          ...
          plugins: ["voxel_layer", "denoise_layer", "inflation_layer"]
          ...
          denoise_layer:
            plugin: "nav2_costmap_2d::DenoiseLayer"
            enabled: true
            minimal_group_size: 2
            group_connectivity_type: 8
