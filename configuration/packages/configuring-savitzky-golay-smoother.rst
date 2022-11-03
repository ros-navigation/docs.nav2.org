.. _configuring_savitzky_golay_filter_smoother:

Savitzky-Golay Smoother
#######################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_smoother

The Savitzky-Golay Smoother is a Smoother Server plugin that will take in an input path and smooth it using a simple and fast smoothing technique based on `Savitzky Golay Filters <https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter>`_. It uses a digital signal processing technique designed to reduce noise distorting a reference signal, in this case, a path.

It is useful for all types of planners, but particularly in NavFn to remove tiny artifacts that can occur near the end of paths or Theta* to slightly soften the transition between Line of Sight line segments **without** modifying the primary path. It is very fast (<< 1ms) so is a recommended default for planners that may result in slight discontinuities. However, it will not smooth out larger scale discontinuities, oscillations, or improve smoothness. For those, use one of the other provided smoother plugins. It also provides estimated orientation vectors of the path points after smoothing.

This algorithm is deterministic and low-parameter. In the below image, some odd points from NavFn's gradient descent are smoothed out by the smoother in the middle and end of a given path, while otherwise retaining the exact character of the path.

.. image:: images/savitzky-golay-example.png
    :align: center

Savitzky-Golay Smoother Parameters
**********************************

:do_refinement:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  bool           True   
  ============== ===========================

  Description
    Whether to smooth the smoothed results ``refinement_num`` times to get an improved result.

:refinement_num:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            2   
  ============== ===========================

  Description
    Number of times to recursively smooth a segment

Example
*******
.. code-block:: yaml

    smoother_server:
      ros__parameters:
        costmap_topic: global_costmap/costmap_raw
        footprint_topic: global_costmap/published_footprint
        robot_base_frame: base_link
        transform_timeout: 0.1
        smoother_plugins: ["savitzky_golay_smoother"]
        savitzky_golay_smoother:
          plugin: "nav2_smoother::SavitzkyGolaySmoother"
          do_refinement: True
          refinement_num: 2
