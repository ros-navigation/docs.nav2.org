.. _configuring_recovery_server:

Recovery Server
###############

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_recoveries

The Recovery Server implements the server for handling recovery requests and hosting a vector of plugins implementing various C++ recoveries.
It is also possible to implement independent recovery servers for each custom recovery, but this server will allow multiple recoveries to share resources such as costmaps and TF buffers to lower incremental costs for new behaviors.

Note: the wait recovery has no parameters, the duration to wait is given in the action request.

Recovery Server Parameters
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

:cycle_frequency:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         10.0 
  ============== =============================

  Description
    Frequency to run recovery plugins.

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

:recovery_plugins:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  vector<string> {"spin", "back_up", "wait"}
  ============== =============================

  Description
    List of plugin names to use, also matches action server names.

  Note
    Each plugin namespace defined in this list needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        recoveries_server:
          ros__parameters:
            recovery_plugins: ["spin", "backup", "wait"]
            spin:
              plugin: "nav2_recoveries/Spin"
            backup:
              plugin: "nav2_recoveries/BackUp"
            wait:
              plugin: "nav2_recoveries/Wait"
    ..

Default Plugins
***************

When the :code:`recovery_plugins` parameter is not overridden, the following default plugins are loaded:

  ================= =====================================================
  Namespace         Plugin
  ----------------- -----------------------------------------------------
  "spin"            "nav2_recoveries/Spin"
  ----------------- -----------------------------------------------------
  "backup"          "nav2_recoveries/BackUp"
  ----------------- -----------------------------------------------------
  "wait"            "nav2_recoveries/Wait"
  ================= =====================================================

Spin Recovery Parameters
************************

Spin distance is given from the action request

:simulate_ahead_time:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         2.0            
  ============== =============================

  Description
    Time to look ahead for collisions (s).

:max_rotational_vel:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         1.0            
  ============== =============================

  Description
    Maximum rotational velocity (rad/s).

:min_rotational_vel:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.4            
  ============== =============================

  Description
    Minimum rotational velocity (rad/s).

:rotational_acc_lim:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         3.2            
  ============== =============================

  Description
    maximum rotational acceleration (rad/s^2).

BackUp Recovery Parameters
**************************

Backup distance is given from the action request.

:simulate_ahead_time:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         2.0            
  ============== =============================

  Description
    Time to look ahead for collisions (s).

Example
*******
.. code-block:: yaml

    recoveries_server:
      ros__parameters:
        costmap_topic: local_costmap/costmap_raw
        footprint_topic: local_costmap/published_footprint
        cycle_frequency: 10.0
        recovery_plugins: ["spin", "backup", "wait"]
        spin:
          plugin: "nav2_recoveries/Spin"
        backup:
          plugin: "nav2_recoveries/BackUp"
        wait:
          plugin: "nav2_recoveries/Wait"
        global_frame: odom
        robot_base_frame: base_link
        transform_timeout: 0.1
        simulate_ahead_time: 2.0
        max_rotational_vel: 1.0
        min_rotational_vel: 0.4
        rotational_acc_lim: 3.2
