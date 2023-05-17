.. _configuring_controller_server:

Controller Server
#################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_controller

The Controller Server implements the server for handling the controller requests for the stack and host a map of plugin implementations.
It will take in path and plugin names for controller, progress checker and goal checker to use and call the appropriate plugins.

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

  Note
    Each plugin namespace defined in this list needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        controller_server:
          ros__parameters:
            controller_plugins: ["FollowPath"]
            FollowPath:
              plugin: "dwb_core::DWBLocalPlanner"
    ..

:progress_checker_plugins:

  ============== ==============
  Type           Default
  -------------- --------------
  vector<string> ["progress_checker"]
  ============== ==============

  Description
    Mapped name for progress checker plugin for checking progress made by robot. Formerly ``progress_checker_plugin`` for Humble and older with a single string plugin.

  Note
    The plugin namespace defined needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        controller_server:
          ros__parameters:
            progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
            progress_checker:
              plugin: "nav2_controller::SimpleProgressChecker"
    ..

:goal_checker_plugins:

  ============== ==============
  Type           Default
  -------------- --------------
  string         'goal_checker'
  ============== ==============

  Description
    Mapped name for goal checker plugin for checking goal is reached.

  Note
    The plugin namespace defined needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        controller_server:
          ros__parameters:
            goal_checker_plugins: ["goal_checker"]
            goal_checker:
              plugin: "nav2_controller::SimpleGoalChecker"

:min_x_velocity_threshold:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.0001            
  ============== =============================

  Description
    The controller server filters the velocity portion of the odometry messages received before sending them to the controller plugin.
    Odometry values below this threshold (in m/s) will be set to 0.0.

:min_y_velocity_threshold:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.0001            
  ============== =============================

  Description
    The controller server filters the velocity portion of the odometry messages received before sending them to the controller plugin.
    Odometry values below this threshold (in m/s) will be set to 0.0. For non-holonomic robots

:min_theta_velocity_threshold:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.0001            
  ============== =============================

  Description
    The controller server filters the velocity portion of the odometry messages received before sending them to the controller plugin.
    Odometry values below this threshold (in rad/s) will be set to 0.0.
    
:failure_tolerance:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.0           
  ============== =============================

  Description
    The maximum duration in seconds the called controller plugin can fail (i.e. the :code:`computeVelocityCommands` function of the plugin throwing an exception) before the :code:`nav2_msgs::action::FollowPath` action fails.
    Setting it to the special value of -1.0 makes it infinite, 0 to disable, and any positive value for the appropriate timeout.

:speed_limit_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "speed_limit"
  ============== =============================

  Description
    Speed limiting topic name to subscribe. This could be published by Speed Filter (please refer to :ref:`speed_filter` configuration page). You can also use this without the Speed Filter as well if you provide an external server to publish `these messages <https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/msg/SpeedLimit.msg>`_.

:odom_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "odom"
  ============== =============================

  Description
    Topic to get instantaneous measurement of speed from.


Provided Plugins
****************
 The plugins listed below are inside the ``nav2_controller`` namespace.

.. toctree::
  :maxdepth: 1

  nav2_controller-plugins/simple_progress_checker.rst
  nav2_controller-plugins/pose_progress_checker.rst
  nav2_controller-plugins/simple_goal_checker.rst
  nav2_controller-plugins/stopped_goal_checker.rst

Default Plugins
***************

When the :code:`progress_checker_plugins`, :code:`goal_checker_plugin` or :code:`controller_plugins` parameters are not overridden, the following default plugins are loaded:

  ================== =====================================================
  Namespace          Plugin
  ------------------ -----------------------------------------------------
  "progress_checker" "nav2_controller::SimpleProgressChecker"
  ------------------ -----------------------------------------------------
  "goal_checker"     "nav2_controller::SimpleGoalChecker"
  ------------------ -----------------------------------------------------
  "FollowPath"       "dwb_core::DWBLocalPlanner"
  ================== =====================================================

Example
*******
.. code-block:: yaml

    controller_server:
      ros__parameters:
        use_sim_time: True
        controller_frequency: 20.0
        min_x_velocity_threshold: 0.001
        min_y_velocity_threshold: 0.5
        min_theta_velocity_threshold: 0.001
        failure_tolerance: 0.3
        odom_topic: "odom"
        progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
        goal_checker_plugin: "goal_checker"
        controller_plugins: ["FollowPath"]
        progress_checker:
          plugin: "nav2_controller::SimpleProgressChecker"
          required_movement_radius: 0.5
          movement_time_allowance: 10.0
        goal_checker:
          plugin: "nav2_controller::SimpleGoalChecker"
          xy_goal_tolerance: 0.25
          yaw_goal_tolerance: 0.25
          stateful: True
        FollowPath:
          plugin: "dwb_core::DWBLocalPlanner"
