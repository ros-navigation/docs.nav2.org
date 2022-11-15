.. _configuring_simple_smoother:

Simple Smoother
###############

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_smoother

The Simple Smoother is a Smoother Server plugin that will take in an input path and smooth it using a simple and fast smoothing technique. It weights the initial path points and the smoothed path points to create a balanced result where the path retains its high level characteristics but reduces oscillations or jagged features.

It is recommended this is paired ONLY with infeasible (e.g. 2D) planners, since this algorithm will break any kinematically feasible conditions. It is recommended users use the Constrained Smoother plugin instead with feasible plans. 

Simple Smoother Parameters
**************************

:tolerance:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         1.0e-10   
  ============== ===========================

  Description
    Change in parameter values across path to terminate smoothing 

:do_refinement:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  bool           True   
  ============== ===========================

  Description
    Whether to smooth the smoothed path recursively to refine the quality further

:refinement_num:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            2   
  ============== ===========================

  Description
    Number of times to recursively attempt to smooth, must be ``>= 1``.

:max_its:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  int            1000   
  ============== ===========================

  Description
    Maximum number of iterations to attempt smoothing before termination

:w_data:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.2   
  ============== ===========================

  Description
    Weight to apply to path data given (bounds it)

:w_smooth:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  double         0.3
  ============== ===========================

  Description
    Weight to apply to smooth the path (smooths it)

Example
*******
.. code-block:: yaml

    smoother_server:
      ros__parameters:
        costmap_topic: global_costmap/costmap_raw
        footprint_topic: global_costmap/published_footprint
        robot_base_frame: base_link
        transform_timeout: 0.1
        smoother_plugins: ["simple_smoother"]
        simple_smoother:
          plugin: "nav2_smoother::SimpleSmoother"
          tolerance: 1.0e-10
          do_refinement: True
          refinement_num: 2
          max_its: 1000
          w_data: 0.2
          w_smooth: 0.3
