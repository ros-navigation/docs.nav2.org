.. _configuring_controller_server:

Controller Server
#################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_controller

The Controller Server implements the server for handling the controller requests for the stack and host a map of plugin implementations.
It will take in a path and a controller plugin name to use and call the appropriate plugin.

Parameters
**********

:controller_frequency:

  ============== =======
  Type           Default
  -------------- -------
  double         20.0   
  ============== =======

  Description
    Frequency to run controller (Hz).

:controller_plugins:

  ============== ==============
  Type           Default                                               
  -------------- --------------
  vector<string> ['FollowPath']            
  ============== ==============

  Description
    List of mapped names for controller plugins for processing requests and parameters.

:min_x_velocity_threshold:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.0001            
  ============== =============================

  Description
    Minimum X velocity to use (m/s).

:min_y_velocity_threshold:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.0001            
  ============== =============================

  Description
    Minimum Y velocity to use (m/s).

:min_theta_velocity_threshold:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.0001            
  ============== =============================

  Description
    Minimum angular velocity to use (rad/s).

:required_movement_radius:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.5            
  ============== =============================

  Description
    Minimum amount a robot must move to be progressing to goal (m).

:movement_time_allowance:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         10.0         
  ============== =============================

  Description
    Maximum amount of time a robot has to move the minimum radius (s).

Default Plugins
***************

When the :code:`controller_plugins` parameter is not overridden, the following default plugins are loaded:

  ================= =====================================================
  Namespace         Plugin
  ----------------- -----------------------------------------------------
  "FollowPath"      "dwb_core::DWBLocalPlanner"
  ================= =====================================================

Example
*******
.. code-block:: yaml

    controller_server:
      ros__parameters:
        controller_frequency: 20.0
        min_x_velocity_threshold: 0.01
        min_y_velocity_threshold: 0.0
        min_theta_velocity_threshold: 0.1
        required_movement_radius: 0.5
        movement_time_allowance: 5.0
        controller_plugins: ['FollowPath']
        FollowPath:
          plugin: "dwb_core::DWBLocalPlanner"
