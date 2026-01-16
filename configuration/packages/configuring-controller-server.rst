.. _configuring_controller_server:

Controller Server
#################

Source code on Github_.

.. _Github: https://github.com/ros-navigation/navigation2/tree/main/nav2_controller

The Controller Server implements the server for handling the controller requests for the stack and host a map of plugin implementations.
It will take in path and plugin names for controller, progress checker and goal checker to use and call the appropriate plugins.
It also hosts the local costmap.

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

:costmap_update_timeout:

  ============== ========
  Type           Default
  -------------- --------
  double         0.3
  ============== ========

  Description
    The timeout value (seconds) for the costmap to be fully updated before a control effort can be computed.

:use_realtime_priority:

  ============== =======
  Type           Default
  -------------- -------
  bool           false
  ============== =======

  Description
    Adds soft real-time prioritization to the controller server to better ensure resources to time sensitive portions of the codebase. This will set the controller's execution thread to a higher priority than the rest of the system (``90``) to meet scheduling deadlines to have less missed loop rates. To use this feature, you use set the following inside of ``/etc/security/limits.conf`` to give userspace access to elevated prioritization permissions: ``<username> soft rtprio 99 <username> hard rtprio 99``

:publish_zero_velocity:

  ============== =======
  Type           Default
  -------------- -------
  bool           true
  ============== =======

  Description
    Whether to publish a zero velocity command on goal exit. This is useful for stopping the robot when a goal terminates.

:controller_plugins:

  ============== ==============
  Type           Default
  -------------- --------------
  vector<string> ['follow_path']
  ============== ==============

  Description
    List of mapped names for controller plugins for processing requests and parameters.

  Note
    Each plugin namespace defined in this list needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        controller_server:
          ros__parameters:
            controller_plugins: ["follow_path"]
            follow_path:
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

  ============== ================
  Type           Default
  -------------- ----------------
  vector<string> ["goal_checker"]
  ============== ================

  Description
    Mapped name for goal checker plugin for checking goal is reached. When the number of the plugins is more than 2, each :code:`follow_path` action needs to specify the goal checker plugin name with its :code:`goal_checker_id` field.

  Note
    The plugin namespace defined needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        controller_server:
          ros__parameters:
            goal_checker_plugins: ["goal_checker"]
            goal_checker:
              plugin: "nav2_controller::SimpleGoalChecker"

:path_handler_plugins:

  ============== ================
  Type           Default
  -------------- ----------------
  vector<string> ["path_handler"]
  ============== ================

  Description
    Mapped name for path handler plugin for processing path from the planner. When the number of the plugins is more than 2, each :code:`follow_path` action needs to specify the path handler plugin name with its :code:`path_handler_id` field.

  Note
    The plugin namespace defined needs to have a :code:`plugin` parameter defining the type of plugin to be loaded in the namespace.

    Example:

    .. code-block:: yaml

        controller_server:
          ros__parameters:
            path_handler_plugins: ["path_handler"]
            path_handler:
              plugin: "nav2_controller::FeasiblePathHandler"


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
    Speed limiting topic name to subscribe. This could be published by Speed Filter (please refer to :ref:`speed_filter` configuration page). You can also use this without the Speed Filter as well if you provide an external server to publish `these messages <https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/msg/SpeedLimit.msg>`_.

:odom_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "odom"
  ============== =============================

  Description
    Topic to get instantaneous measurement of speed from.

:odom_duration:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  double         0.3
  ============== ===========================

  Description
    Time (s) to buffer odometry commands to estimate the robot speed.

:enable_stamped_cmd_vel:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           true
  ============== =============================

  Description
    Whether to use geometry_msgs::msg::Twist or geometry_msgs::msg::TwistStamped velocity data.
    True uses TwistStamped, false uses Twist.
    Note: This parameter is default ``false`` in Jazzy or older! Kilted or newer uses ``TwistStamped`` by default.

:bond_heartbeat_period:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.25
  ============== =============================

  Description
    The lifecycle node bond mechanism publishing period (on the /bond topic). Disabled if inferior or equal to 0.0.

:introspection_mode:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "disabled"
  ============== =============================

  Description
    The introspection mode for services and actions. Options are "disabled", "metadata", "contents".

:allow_parameter_qos_overrides:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           true
  ============== =============================

  Description
    Whether to allow QoS profiles to be overwritten with parameterized values.

:search_window:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double           2.0
  ============== =============================

  Description
    How far (in meters) along the path the searching algorithm will look for the closest point.

Provided Plugins
****************
 The plugins listed below are inside the ``nav2_controller`` namespace.

.. toctree::
  :maxdepth: 1

  nav2_controller-plugins/simple_progress_checker.rst
  nav2_controller-plugins/pose_progress_checker.rst
  nav2_controller-plugins/axis_goal_checker.rst
  nav2_controller-plugins/simple_goal_checker.rst
  nav2_controller-plugins/stopped_goal_checker.rst
  nav2_controller-plugins/position_goal_checker.rst
  nav2_controller-plugins/feasible_path_handler.rst

Default Plugins
***************

When the :code:`progress_checker_plugins`, :code:`goal_checker_plugin`, :code:`path_handler_plugin` or :code:`controller_plugins` parameters are not overridden, the following default plugins are loaded:

  ================== =====================================================
  Namespace          Plugin
  ------------------ -----------------------------------------------------
  "progress_checker" "nav2_controller::SimpleProgressChecker"
  ------------------ -----------------------------------------------------
  "goal_checker"     "nav2_controller::SimpleGoalChecker"
  ------------------ -----------------------------------------------------
  "path_handler"     "nav2_controller::FeasiblePathHandler"
  ------------------ -----------------------------------------------------
  "follow_path"       "dwb_core::DWBLocalPlanner"
  ================== =====================================================

Example
*******
.. code-block:: yaml

    controller_server:
      ros__parameters:
        controller_frequency: 20.0
        costmap_update_timeout: 0.3
        min_x_velocity_threshold: 0.001
        min_y_velocity_threshold: 0.5
        min_theta_velocity_threshold: 0.001
        failure_tolerance: 0.3
        odom_topic: "odom"
        odom_duration: 0.3
        progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
        goal_checker_plugins: ["goal_checker"] # goal_checker_plugin: "goal_checker" For Galactic and older
        path_handler_plguins: ["path_handler"]
        controller_plugins: ["follow_path"]
        progress_checker:
          plugin: "nav2_controller::SimpleProgressChecker"
          required_movement_radius: 0.5
          movement_time_allowance: 10.0
        goal_checker:
          plugin: "nav2_controller::SimpleGoalChecker"
          xy_goal_tolerance: 0.25
          yaw_goal_tolerance: 0.25
          path_length_tolerance: 1.0
          stateful: True
        path_handler:
          plugin: "nav2_controller::FeasiblePathHandler"
          prune_distance: 2.0
          enforce_path_inversion: True
          enforce_path_rotation: False
          inversion_xy_tolerance: 0.2
          inversion_yaw_tolerance: 0.4
          minimum_rotation_angle: 0.785
          reject_unit_path: False
        follow_path:
          plugin: "dwb_core::DWBLocalPlanner"
