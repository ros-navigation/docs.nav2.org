.. _configuring_smoother_server:

Smoother Server
###############

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_smoother

The Smoother Server implements the server for handling smooth path requests and hosting a vector of plugins implementing various C++ smoothers.
The server allows multiple smoothers to share resources such as costmaps and TF buffers to lower incremental costs for new behaviors.

Smoother Server Parameters
**************************

:costmap_topic:

  ============== ===========================
  Type           Default                    
  -------------- ---------------------------
  string         "local_costmap/costmap_raw"   
  ============== ===========================

  Description
    Raw costmap topic for collision checking.

:footprint_topic:

  ============== ===================================
  Type           Default                                               
  -------------- -----------------------------------
  string         "local_costmap/published_footprint"            
  ============== ===================================

  Description
    Topic for footprint in the costmap frame.

:transform_tolerance:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.1 
  ============== =============================

  Description
    TF transform tolerance.

:global_frame:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "odom" 
  ============== =============================

  Description
    Reference frame.

:robot_base_frame:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  string         "base_link" 
  ============== =============================

  Description
    Robot base frame.

:smoother_plugins:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  vector<string> {}
  ============== =============================

  Description
    List of plugin names to use, also matches action server names.

  Note
    Each plugin namespace defined in this list needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        smoother_server:
          ros__parameters:
            smoother_plugins: ["SmoothPath"]
            SmoothPath:
              plugin: "nav2_ceres_costaware_smoother/CeresCostawareSmoother"
    ..

Default Plugins
***************

When the :code:`smoother_plugins` parameter is not overridden, the following default plugins are loaded:

  ================= =====================================================
  Namespace         Plugin
  ----------------- -----------------------------------------------------
  ================= =====================================================

Example
*******
.. code-block:: yaml

    smoother_server:
      ros__parameters:
        costmap_topic: global_costmap/costmap_raw
        footprint_topic: global_costmap/published_footprint
        global_frame: map
        robot_base_frame: base_link
        transform_timeout: 0.1
        smoother_plugins: ["SmoothPath"]

        SmoothPath:
          plugin: "my_plugin_package/MyPlugin"
          minimum_turning_radius: 0.5
          smoothing_intensity: 10.0
          my_other_param: 1.0
