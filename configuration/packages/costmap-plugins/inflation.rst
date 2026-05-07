.. inflation:

Inflation Layer Parameters
==========================

This layer places an exponential decay functions around obstacles to increase cost to traverse near collision. It also places a lethal cost around obstacles within the robot's fully inscribed radius - even if a robot is non-circular for optimized first-order collision checking.

``<inflation layer>`` is the corresponding plugin name selected for this type.


:``<inflation layer>``.enabled:

  ==== =======
  Type Default
  ---- -------
  bool True
  ==== =======

  Description
    Whether it is enabled.

:``<inflation layer>``.inflation_radius:

  ====== =======
  Type   Default
  ------ -------
  double 0.55
  ====== =======

  Description
    Radius to inflate costmap around lethal obstacles.

:``<inflation layer>``.cost_scaling_factor:

  ====== =======
  Type   Default
  ------ -------
  double 10.0
  ====== =======

  Description
    Exponential decay factor across inflation radius.


:``<inflation layer>``.inflate_unknown:

  ==== =======
  Type Default
  ---- -------
  bool False
  ==== =======

  Description
    Whether to inflate unknown cells as if lethal.


:``<inflation layer>``.inflate_around_unknown:

  ==== =======
  Type Default
  ---- -------
  bool False
  ==== =======

  Description
    Whether to inflate unknown cells.


:``<inflation layer>``.num_threads:

  ==== =======
  Type Default
  ---- -------
  int  -1
  ==== =======

  Description
    Number of threads to use for inflation computation when OpenMP is enabled.
    Set to ``-1`` for auto-detection (uses half of available CPU cores), or specify a positive integer for explicit thread count.
    Ignored if OpenMP support is not available.

Building with OpenMP
--------------------

OpenMP parallelization is disabled by default. To enable it, pass the ``ENABLE_OPENMP`` CMake option when building:

.. code-block:: bash

    # Build with OpenMP enabled
    colcon build --packages-select nav2_costmap_2d --cmake-args -DENABLE_OPENMP=ON

    # Build with OpenMP disabled (default)
    colcon build --packages-select nav2_costmap_2d --cmake-args -DENABLE_OPENMP=OFF

Performance Benchmarks
----------------------

**Test Configuration:** 2000×2000 Grid (4M cells, 50% occupancy, 2m inflation radius)

**Robot Hardware:** 16 cores × 5000 MHz

+---------------------+----------+------------------+---------------------------+
| Configuration       | Time     | Throughput       | vs Legacy Inflation Layer |
+=====================+==========+==================+===========================+
| OpenMP disabled     | 48.9 ms  | 81.8 M cells/s   | 2.1× faster               |
+---------------------+----------+------------------+---------------------------+
| OpenMP enabled      | 9.11 ms  | 468.9 M cells/s  | 11.5× faster              |
+---------------------+----------+------------------+---------------------------+
