.. _configuring_docking_server:

Docking Server
##############

Source code on Github_.

.. _Github: https://github.com/open-navigation/opennav_docking

The Docking Server in ``opennav_docking`` implements a server for docking and undocking a robot.
This can be from Charging stations (i.e. docks) or non-charging docking locations such as the end of a conveyor belt or a pallet.
It uses plugin `dock` implementations for a particular platform to enable the framework to generalize to robots of many different kinematic models, charging methods, sensor modalities, charging-type, and so on.
It can also handle a database of many different docking locations and dock models to handle a heterogeneous environment.
This task server is designed be called by an application BT or autonomy application to dock once completed with tasks or battery is low -- not within the navigate-to-pose action itself (though `undock` may be called from inside navigate actions!).

Thanks to NVIDIA for sponsoring this Docking Server package!

Parameters
**********

:controller_frequency:

  ============== ==============
  Type           Default
  -------------- --------------
  double         50.0
  ============== ==============

  Description
    Control frequency (Hz) for vision-control loop.

:initial_perception_timeout:

  ============== ==============
  Type           Default
  -------------- --------------
  double         5.0
  ============== ==============

  Description
    Timeout (s) to wait to obtain initial perception of the dock.

:wait_charge_timeout:

  ============== ==============
  Type           Default
  -------------- --------------
  double         5.0
  ============== ==============

  Description
    Timeout (s) to wait to see if charging starts after docking.

:dock_approach_timeout:

  ============== ==============
  Type           Default
  -------------- --------------
  double         30.0
  ============== ==============

  Description
    Timeout (s) to attempt vision-control approach loop.

:undock_linear_tolerance:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.05
  ============== ==============

  Description
    Tolerance (m) to exit the undocking control loop at staging pose.

:undock_angular_tolerance:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.05
  ============== ==============

  Description
    Angular tolerance (rad) to exit undocking loop at staging pose.

:rotation_angular_tolerance:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.05
  ============== ==============

  Description
    Angular tolerance (rad) to exit the rotation loop when rotate_to_dock is enabled.

:max_retries:

  ============== ==============
  Type           Default
  -------------- --------------
  int            3
  ============== ==============

  Description
    Maximum number of retries to attempt.

:base_frame:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "base_link"
  ============== ==============

  Description
    Robot's base frame for control law.

:fixed_frame:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "odom"
  ============== ==============

  Description
    Fixed frame to use, recommended to be a smooth odometry frame **not** map.

:odom_topic:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "odom"
  ============== ==============

  Description
    The topic to use for the odometry data when rotate_to_dock is enabled.

:dock_backwards:

  ============== ==============
  Type           Default
  -------------- --------------
  bool           false
  ============== ==============

  Description
    Whether the robot is docking with the dock forward or backward in motion. This parameter is deprecated. Use the dock plugin's ``dock_direction`` parameter instead.

:dock_prestaging_tolerance:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.5
  ============== ==============

  Description
    L2 distance in X,Y,Theta from the staging pose to bypass navigation.

:dock_plugins:

  ============== ==============
  Type           Default
  -------------- --------------
  vector<string> N/A
  ============== ==============

  Description
    A set of dock plugins to load.

:dock_database:

  ============== ==============
  Type           Default
  -------------- --------------
  string         N/A
  ============== ==============

  Description
    The filepath to the dock database to use for this environment. Use ``docks`` or this param.


:docks:

  ============== ==============
  Type           Default
  -------------- --------------
  vector<string> N/A
  ============== ==============

  Description
    Instead of `dock_database`, the set of docks specified in the params file itself. Use ``dock_database`` or this param.

:navigator_bt_xml:

  ============== ==============
  Type           Default
  -------------- --------------
  string         ""
  ============== ==============

  Description
    BT XML to use for Navigator, if non-default.

:service_introspection_mode:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "disabled"
  ============== =============================

  Description
    The introspection mode for services. Options are "disabled", "metadata", "contents".

:controller.k_phi:

  ============== ==============
  Type           Default
  -------------- --------------
  double         3.0
  ============== ==============

  Description
    Ratio of the rate of change of angle relative to distance from the target. Much be > 0.

:controller.k_delta:

  ============== ==============
  Type           Default
  -------------- --------------
  double         2.0
  ============== ==============

  Description
    Higher values result in converging to the target more quickly.

:controller.beta:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.4
  ============== ==============

  Description
    Parameter to reduce linear velocity proportional to path curvature. Increasing this linearly reduces the velocity (v(t) = v_max / (1 + beta * \|curv\|^lambda)).

:controller.lambda:

  ============== ==============
  Type           Default
  -------------- --------------
  double         2.0
  ============== ==============

  Description
    Parameter to reduce linear velocity proportional to path curvature. Increasing this exponentially reduces the velocity (v(t) = v_max / (1 + beta * \|curv\|^lambda)).

:controller.v_linear_min:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.1
  ============== ==============

  Description
    Minimum velocity for approaching dock.

:controller.v_linear_max:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.24
  ============== ==============

  Description
    Maximum velocity for approaching dock.

:controller.v_angular_max:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.75
  ============== ==============

  Description
    Maximum angular velocity for approaching dock.

:controller.slowdown_radius:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.25
  ============== ==============

  Description
    Radius to end goal to commense slow down.

:controller.rotate_to_heading_angular_vel:

  ============== ==============
  Type           Default
  -------------- --------------
  double         1.0
  ============== ==============

  Description
    Angular velocity (rad/s) to rotate to the goal heading when rotate_to_dock is enabled.

:controller.rotate_to_heading_max_angular_accel:

  ============== ==============
  Type           Default
  -------------- --------------
  double         3.2
  ============== ==============

  Description
    Maximum angular acceleration (rad/s^2) to rotate to the goal heading when rotate_to_dock is enabled.

:controller.use_collision_detection:

  ============== ==============
  Type           Default
  -------------- --------------
  bool           true
  ============== ==============

  Description
    Whether to use collision detection to avoid obstacles.

:controller.costmap_topic:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  string         "local_costmap/costmap_raw"
  ============== ===========================

  Description
    Raw costmap topic for collision checking.

:controller.footprint_topic:

  ============== ===================================
  Type           Default
  -------------- -----------------------------------
  string         "local_costmap/published_footprint"
  ============== ===================================

  Description
    Topic for footprint in the costmap frame.

:controller.transform_tolerance:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.1
  ============== =============================

  Description
    Time with which to post-date the transform that is published, to indicate that this transform is valid into the future.

:controller.projection_time:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         1.0
  ============== =============================

  Description
    Time to look ahead for collisions (s).

:controller.simulation_time_step:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.1
  ============== =============================

  Description
    Time step for projections (s).

:controller.dock_collision_threshold:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.3
  ============== =============================

  Description
    Distance (m) from the dock pose to ignore collisions, i.e. the robot will not check for collisions within this distance from the dock pose, as the robot will make contact with the dock. Set to ``0.0`` when physical contact is not made with a dock.


Note: ``dock_plugins`` and either ``docks`` or ``dock_database`` are required.


SimpleChargingDock Parameters
*****************************

Simple Charging Dock is a provided charging dock plugin that can handle many docks and common techniques.

:<dock_name>.staging_x_offset:

  ============== ==============
  Type           Default
  -------------- --------------
  double         -0.7
  ============== ==============

  Description
    Staging pose offset forward (negative) of dock pose (m).

:<dock_name>.staging_yaw_offset:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.0
  ============== ==============

  Description
    Staging pose angle relative to dock pose (rad). If ``dock_direction`` is set to "backward", this angle must be faced in the opposite direction of the dock pose. However, if ``rotate_to_dock`` is enabled, this angle must be facing the same direction as the dock pose because the robot will rotate to the dock pose after detection.

:<dock_name>.use_battery_status:

  ============== ==============
  Type           Default
  -------------- --------------
  bool           true
  ============== ==============

  Description
    Whether to use the battery state message or ``isDocked()`` for ``isCharging()``.

:<dock_name>.use_external_detection_pose:

  ============== ==============
  Type           Default
  -------------- --------------
  bool           false
  ============== ==============

  Description
    Whether to use external detection topic for dock or use the databases' pose.

:<dock_name>.external_detection_timeout:

  ============== ==============
  Type           Default
  -------------- --------------
  double         1.0
  ============== ==============

  Description
    Timeout (s) at which if the newest detection update does not meet to fail.


:<dock_name>.external_detection_translation_x:

  ============== ==============
  Type           Default
  -------------- --------------
  double         -0.20
  ============== ==============

  Description
    X offset from detected pose for docking pose (m).

:<dock_name>.external_detection_translation_y:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.0
  ============== ==============

  Description
    Y offset from detected pose for docking pose (m).

:<dock_name>.external_detection_rotation_yaw:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.0
  ============== ==============

  Description
    Yaw offset from detected pose for docking pose (rad).

:<dock_name>.external_detection_rotation_pitch:

  ============== ==============
  Type           Default
  -------------- --------------
  double         1.57
  ============== ==============

  Description
    Pitch offset from detected pose for docking pose (rad). Note: The external detection rotation angles are setup to work out of the box with Apriltags detectors in `image_proc` and `isaac_ros`.

:<dock_name>.external_detection_rotation_roll:

  ============== ==============
  Type           Default
  -------------- --------------
  double         -1.57
  ============== ==============

  Description
    Roll offset from detected pose for docking pose (rad). Note: The external detection rotation angles are setup to work out of the box with Apriltags detectors in `image_proc` and `isaac_ros`.

:<dock_name>.filter_coef:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.1
  ============== ==============

  Description
    Dock external detection method filtering algorithm coefficient.

:<dock_name>.charging_threshold:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.5
  ============== ==============

  Description
    Threshold of current in battery state above which ``isCharging() = true``.

:<dock_name>.use_stall_detection:

  ============== ==============
  Type           Default
  -------------- --------------
  bool           false
  ============== ==============

  Description
    Whether or not to use stall detection for ``isDocked()`` or positional threshold.

:<dock_name>.stall_joint_names:

  ============== ==============
  Type           Default
  -------------- --------------
  vector<string> N/A
  ============== ==============

  Description
    Names in ``joint_states`` topic of joints to track.

:<dock_name>.stall_velocity_threshold:

  ============== ==============
  Type           Default
  -------------- --------------
  double         1.0
  ============== ==============

  Description
    The joint velocity below which to trigger ``isDocked() = true``.

:<dock_name>.stall_effort_threshold:

  ============== ==============
  Type           Default
  -------------- --------------
  double         1.0
  ============== ==============

  Description
    Current or motor effort in joint state to trigger ``isDocked() = true``.

:<dock_name>.docking_threshold:

  ============== ==============
  Type           Default
  -------------- --------------
  double         0.05
  ============== ==============

  Description
    If not using stall detection, the pose threshold to the docking pose where ``isDocked() = true``.

:<dock_name>.dock_direction:

  ============== ==============
  Type           Default
  -------------- --------------
  string         "forward"
  ============== ==============

  Description
    Whether the robot is docking with the dock forward or backward in motion. This is the replacement for the deprecated ``dock_backwards`` parameter. Options are "forward" or "backward".

:<dock_name>.rotate_to_dock:

  ============== ==============
  Type           Default
  -------------- --------------
  bool           false
  ============== ==============

  Description
    Enables backward docking without requiring a sensor for detection during the final approach. When enabled, the robot approaches the staging pose facing forward with sensor coverage for dock detection; after detection, it rotates and backs into the dock using only the initially detected pose for dead reckoning. In the undocking phase, the robot will move forward to the staging pose and then rotate to the original heading. This may also be paired with sensor detection in the reverse direction as well if available.

    Note: This parameter is only valid when the ``dock_direction`` is set to "backward".

Example
*******
.. code-block:: yaml

    docking_server:
      ros__parameters:
        controller_frequency: 50.0
        initial_perception_timeout: 5.0
        wait_charge_timeout: 5.0
        dock_approach_timeout: 30.0
        undock_linear_tolerance: 0.05
        undock_angular_tolerance: 0.1
        max_retries: 3
        base_frame: "base_link"
        fixed_frame: "odom"
        odom_topic: "odom"
        dock_backwards: false  # Deprecated, use dock_direction in plugin
        dock_prestaging_tolerance: 0.5
        service_introspection_mode: "disabled"

        # Types of docks
        dock_plugins: ['nova_carter_dock']
        nova_carter_dock:
          plugin: 'opennav_docking::SimpleChargingDock'  # Also 'opennav_docking::SimpleNonChargingDock'
          docking_threshold: 0.05
          staging_x_offset: -0.7
          use_external_detection_pose: true
          use_battery_status: false # true
          use_stall_detection: false
          rotate_to_dock: false

          external_detection_timeout: 1.0
          external_detection_translation_x: -0.18
          external_detection_translation_y: 0.0
          external_detection_rotation_roll: -1.57
          external_detection_rotation_pitch: -1.57
          external_detection_rotation_yaw: 0.0
          filter_coef: 0.1
          dock_direction: "forward" # "backward"

        # Dock instances
        docks: ['home_dock']
        home_dock:
          type: 'nova_carter_dock'
          frame: map
          pose: [0.0, 0.0, 0.0]
          id: 'c67f50cb-e152-4720-85cc-5eb20bd85ce8'

        controller:
          k_phi: 3.0
          k_delta: 2.0
          v_linear_min: 0.15
          v_linear_max: 0.15
          v_angular_max: 0.75
          slowdown_radius: 0.25
          rotate_to_heading_angular_vel: 1.0
          rotate_to_heading_max_angular_accel: 3.2
          use_collision_detection: true
          costmap_topic: "local_costmap/costmap_raw"
          footprint_topic: "local_costmap/published_footprint"
          transform_tolerance: 0.1
          projection_time: 1.0
          simulation_time_step: 0.1
          dock_collision_threshold: 0.3
