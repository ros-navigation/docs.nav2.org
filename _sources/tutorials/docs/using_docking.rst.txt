.. _docking_tutorial:

Using Docking Server
********************

- `Overview`_
- `Requirements`_
- `ChargingDock Plugins`_
- `Dock Database`_
- `Configuring Docking Server`_
- `Adding Docking Server to Launch`_
- `Putting It All Together`_

Overview
========

This tutorial shows how to use the Docking Server with Nav2 robot systems.
The Docking Server is a general framework which can be used with arbitrary types of robots and docks in order to auto-dock them.
This is accomplished via plugins ``ChargingDock`` and ``NonChargingDock`` which implement the dock specifics like detecting the pose of the dock using sensor data, how to detect when the robot is in contact with the dock, and when charging has successfully started.
A configuration of the docking server can contain a database of many docks of different plugin ``ChargingDock`` and ``NonChargingDock`` types to handle a broad range of docking locations and hardware dock revisions.
Included with the package is an example ``SimpleChargingDock`` and ``SimpleNonChargingDock`` plugins which contains features and methods very common for robot docking.
These support charging stations and docking with static infrastructure (ex. conveyor belts) or dynamic docking (ex pallets) locations.
It is likely that you may be able to use this as well rather than developing your own dock plugin to get started. 

The docking procedure is as follows:

1. Take action request and obtain the dock's plugin and its pose
2. If the robot is not within the prestaging tolerance of the dock's staging pose, navigate to the staging pose
3. Use the dock's plugin to initially detect the dock and return the docking pose
4. Enter a vision-control loop where the robot attempts to reach the docking pose while its actively being refined by the vision system
5. Exit the vision-control loop once contact has been detected or charging has started 
6. Wait until charging starts (if applicable) and return success.

Thanks to NVIDIA for sponsoring this Docking Server package and this tutorial!
You can find how to dock your Nova Carter robot using Nav2 and this work in the `nova_carter_docking package <https://github.com/open-navigation/opennav_docking/tree/main/nova_carter_docking>`_!

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="708" height="400" src="https://www.youtube.com/embed/leiGkSVnQak?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
      </div>
    </h1>


Requirements
============

It is assumed ROS2 and Nav2 dependent packages are installed or built locally - including ``opennav_docking``.
Please make sure that Nav2 project is also built locally, see :ref:`build-instructions` for reference.

See ``opennav_docking`` README for complete concept explanations, parameters, and APIs.

ChargingDock Plugins
====================

``opennav_docking_core::ChargingDock`` and ``opennav_docking_core::ChargingDock`` plugins are established to abstract out robot- and dock-specifics from the generalized framework.
This allows a system to leverage this framework and provide its own methods for detecting the dock's current pose, when the robot is charging, and when contact is made.
Luckily, there are several common ROS APIs that allow us to create semi-generalized ``SimpleChargingDock`` and ``SimpleNonChargingDock`` plugins that allows out-of-the-box docking as long as users provide ``JointState``, ``BatteryState``, and detected dock pose ``PoseStamped`` topics.
However, one way or another, your system requires an applicable ``ChargingDock`` or ``NonChargingDock`` plugin for each type of dock you wish to use.

The plugins has a few key APIs:

- ``PoseStamped getStagingPose(const Pose & pose, const string & frame)`` which must provide the pre-docking staging pose given a dock's location and frame.
- ``bool getRefinedPose(PoseStamped & pose)`` which must provide the detected (or pass through) pose of the dock 
- ``bool isDocked()`` which provides if we've made contact with the dock
- ``bool isCharging()`` which provides if we've started charging while docked (charging docks only)
- ``bool disableCharging()`` which should disable charging, if under the robot's control for undocking (charging docks only)
- ``bool hasStoppedCharging()`` which indicates if we've successfully stopped charging on undocking (charging docks only)

The ``SimpleChargingDock`` provides an implementation with common options for these APIs:

- ``getStagingPose`` - Finds a relative offset pose with translation and rotation from the dock's pose
- ``getRefinedPose`` - Filters a detected pose topic of type ``PoseStamped`` into the fixed frame *or* is a pass through function returning the dock's database location if detection is not enabled
- ``isDocked`` - Returns as dock if a pose tolerance is met relative to the dock *or* if the ``JointStates`` of the motors detect a clear spike due to stalling by driving into the dock's surface, if enabled
- ``isCharging`` - Returns charging if ``isDocked`` *or* if ``BatteryState``'s current is above a threshold, if enabled (charging docks only)
- ``disableCharging`` - Always ``true``, considers disable of charging as automatic when robot leaves dock (charging docks only)
- ``hasStoppedCharging`` - The inverse of ``isCharging`` (charging docks only)

Thus, for testing (no detection, no battery information, no joint state information) and realistic application (dock detection, battery status information, joint state information), this dock plugin can be used.
It can also be used when only some of the information if available as well. 
If your robot or dock does not fall into these implementations (i.e. using custom battery or detection messages that cannot be converted into ROS standard types), then you may be required to build your own plugin to meet your particular needs.
However, you can use the ``SimpleChargingDock`` assuming you turn off these settings and dock blind to get started.
There is an equivalent ``SimpleNonChargingDock`` plugin for non-charging docking needs.

If you do not currently have a way to detect your dock, dock detection can be done easily using Apriltags and the `isaac_ros_apriltag <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag>`_ or `ROS image_proc <https://github.com/ros-perception/image_pipeline/blob/rolling/image_proc/src/track_marker.cpp>`_ nodes to get started.
Use the Isaac ROS if using a Jetson platform to obtain a GPU optimized pipeline with your camera feeds.
The defaults support this out of the box, see ``nova_carter_docking`` for an example.

.. note::
  It is important to note that you should provide detected dock poses, battery status information for charging, and motor controller efforts for the highest quality and reliable docking for production use.

Dock Database
=============

To dock your robot, you must provide the set of docks in your environment you'd like to utilize.
This is done in the docking server via the *Dock Database* which contains the set of docks, their instance types, and a set of shared plugins.
The plugins are separated from the dock instances so that many instances can share the same plugin to save on memory and networking overhead when potentially dozens or more docks exist in a space.

The docks plugins must be provided in your docking server's configuration file.
However, the dock instance may be provided either in the configuration file *or* within a provided filepath to decouple the server's configuration from a particular application environment.
The example below shows an inline configuration of the docking plugins and dock instances where one dock type (``nova_carter_dock``) is specified with 3 individual instances: a home dock, and 2 general shared fallback docks.
The docks can be specified as ``[x, y, theta]`` in any reference frame you like, as long as TF is aware of them.
Please update these with your own docking plugin and dock locations in your map.

.. code-block:: yaml

  docking_server:
    ros__parameters:
      # Types of docks
      dock_plugins: ['nova_carter_dock']
      nova_carter_dock:
        plugin: 'opennav_docking::SimpleChargingDock'
        # More parameters exist here that we will discuss later in the tutorial

      # Dock instances
      docks: ['home_dock','flex_dock1', 'flex_dock2']
      home_dock:
        type: 'nova_carter_dock'
        frame: map
        pose: [0.0, 0.0, 0.0]
      flex_dock1:
        type: 'nova_carter_dock'
        frame: map
        pose: [10.0, 10.0, 0.0]
      flex_dock2:
        type: 'nova_carter_dock'
        frame: map
        pose: [30.0, 30.0, 0.0]

      # Or use
      # dock_database: /my/path/to/dock_database.yaml

The analog of this is shown below as an independent ``dock_database.yaml`` which can be provided to the ``docking_server`` via the ``dock_database`` parameter.

.. code-block:: yaml

  docks:
    home_dock:
      type: "nova_carter_dock"
      frame: "map"
      pose: [0.0, 0.0, 0.0]
    flex_dock1:
      type: "nova_carter_dock"
      frame: "map"
      pose: [10.0, 10.0, 0.0]
    flex_dock2:
      type: "nova_carter_dock"
      frame: "map"
      pose: [20.0, 20.0, 0.0]

Note that you are required to provide at least 1 dock plugin and 1 dock instance.
The Docking Server's Action API can take in a dock's instance information separately to bypass the database, but its plugin must exist in the server's configuration.
If you plan to only use this API, you can set a ``dummy_dock``.
Generally speaking, its wise to set your docks in the database and use the Docking Server's API to dock at an instance's Dock ID to decouple the semantic information about docks from the action request (requiring your application instead to have all of the docks' locations), but bypassing the database can be useful for testing and movable docking targets.

The dock poses in the map can be annotated using your favorite map editing tools, obtained by ``/clicked_point`` in rviz2, or measured location.


Configuring Docking Server
==========================

Now that we have both a plugin for interacting with a dock and specified the locations of docks in your map, we're ready to configure the docking server.
For this example, we're going to use the Nvidia-Segway Nova Carter Robot and you can find the source code of this demo in the ``nova_carter_docking`` package.
For a full list of parameters and their descriptions, check out the :ref:`configuring_docking_server`.

Below is an example configuration used by the Nova Carter robot.
Notable is the setting of the ``fixed_frame`` to ``odom``, not ``map`` in order to decouple localization error from the docking procedure.
We also use one dock plugin ``nova_carter_dock`` for all ``N`` docks specified in the ``dock_database`` file.

The simple charging dock plugin uses a 70cm staging offset from the dock's database pose for staging.
This staging pose is selected as close enough to detect the dock but far enough away to give maneuvering space to account for expected dock movement or localization error.

Since ``use_stall_detection`` for the ``JointStates`` is ``false``, we are considered successfully docked once we're within ``docking_threshold`` (5cm) to the docking pose.
This docking pose is specified as the detected pose with the ``external_detection_*`` offsets applied to account for the robot's intended docking pose relative to the detected feature.
In this example, Apriltags are used, so we apply the rotations to the Apriltag detected frame and a ``-0.18`` translational offset to account for the pose the robot should be in when docked relative to the tag's pose.
Since ``use_external_detection_pose`` and ``use_battery_status`` are both enabled, we use both detected dock poses (apriltag) and battery state information for determining if we're charging.

The maximum speed is 15 cm/s to slowly and carefully back into the dock and we'll retry docking 3x in case charging is not detected or we lose detected dock tracks during the attempt.

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
        dock_backwards: false
        dock_prestaging_tolerance: 0.5

        # Types of docks
        dock_plugins: ['nova_carter_dock']
        nova_carter_dock:
          plugin: 'opennav_docking::SimpleChargingDock'
          docking_threshold: 0.05
          staging_x_offset: -0.7
          use_external_detection_pose: true
          use_battery_status: true
          use_stall_detection: false

          external_detection_timeout: 1.0
          external_detection_translation_x: -0.18
          external_detection_translation_y: 0.0
          external_detection_rotation_roll: -1.57
          external_detection_rotation_pitch: -1.57
          external_detection_rotation_yaw: 0.0
          filter_coef: 0.1

        # Sep. file of dock instances so config file can be used in multiple locations
        dock_database: /my/path/to/dock_database.yaml

        controller:
          k_phi: 3.0
          k_delta: 2.0
          v_linear_min: 0.15
          v_linear_max: 0.15

Adding Docking Server to Launch
===============================

This server can now be added to your launch file with the path to this parameter file for use (or added to your main shared configuration file).

.. code-block:: python

    nova_carter_dock_params_dir = os.path.join(
            get_package_share_directory('nova_carter_docking'), 'params')
    params_file = default_value=os.path.join(nova_carter_dock_params_dir, 'nova_carter_docking.yaml')

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[params_file],
    )

.. Note::
  The Docking Server is also a composible node like others in Nav2, so you may also launch it within the Nav2 process using ``LoadComposableNodes/ComposableNode``.

Docking Action API
==================

The API for docking and undocking is comparatively simple.

The ``DockRobot`` action consists of two main modalities: using the dock database or specifying a dock to use in the action.
If using the database, set ``use_dock_id = True`` (default) and you only need to specify the ``dock_id`` you wish to use, such as ``home_dock``, ``flex_dock1``, or whatever dock instance you like.
If bypassing the database, ``use_dock_id`` must be set to false and ``dock_pose``, ``dock_type`` must be fully specified to make up for the lack of entry metadata in the database.
This requires the action caller to know about all of the docks, rather than pushing that into the Docking Server's database, which is not recommended.

Optionally, you can disable using Nav2 to navigate to the staging pose if outside of the pre-staging tolerance using ``navigate_to_staging_pose = False`` or set the maximum time for staging navigation ``max_staging_time``.

.. code-block:: bash

  #goal definition
  bool use_dock_id True  # Whether to use the dock_id or dock_pose fields
  string dock_id  # Dock name or ID to dock at, from given dock database

  geometry_msgs/PoseStamped dock_pose  # Dock pose
  string dock_type  # If using dock_pose, what type of dock it is. Not necessary if only using one type of dock.

  float32 max_staging_time 1000.0  # Maximum time for navigation to get to the dock's staging pose.
  bool navigate_to_staging_pose True  # Whether or not to navigate to staging pose or assume robot is already at staging pose within tolerance to execute behavior

  ---
  #result definition
  bool success True  # docking success status
  uint16 error_code 0  # Contextual error code, if any
  uint16 num_retries 0  # Number of retries attempted

  ---
  #feedback definition
  uint16 state  # Current docking state
  builtin_interfaces/Duration docking_time  # Docking time elapsed
  uint16 num_retries 0  # Number of retries attempted

In result, you obtain if the action was successful, if it wasn't what the error code was, and the total number of retries attempted.
During execution, feedback is provided on the current docking state - which is published irregularly only when an event occurs. It contains the state, the current total elapsed duration of attempted docking, and the current number of retries.
The feedback can be obtained from your action client if this information is useful to your application.

The ``UndockRobot`` action is even simpler. There are no required goal fields except ``dock_type`` if undocking is being called when the server's instance did not dock the robot to store its current state information (such as after a restart on the dock).
It contains no feedback and returns the ``success`` state and the ``error_code`` if a problem occurs. 

.. code-block:: bash

  #goal definition
  string dock_type
  float32 max_undocking_time 30.0 # Maximum time to undock

  ---
  #result definition
  bool success True  # docking success status
  uint16 error_code 0  # Contextual error code, if any

  ---
  #feedback definition


Putting It All Together
=======================

At this point, if you haven't already, create your dock plugin (or use ``SimpleChargingDock``), configuration file, and launch file - along with any other nodes required like apriltags or other detectors.
You can see an example package used in this tutorial in the ``nova_carter_docking`` package, which contains a configuration file and launch file containing the apriltags detector and ``PoseStamped`` pose publisher.

If you're interested in using Apriltags and an Nvidia Jetson, you can find the tags we used in the ``media/`` directory and the launch file ``isaac_apriltag_detection_pipeline.launch.py`` which sets it all up for you. 
If not using the Jetson, you can replace the Isaac ROS apriltag detector with ``image_proc``. 

We can test this using the script ``demo.py`` in ``nova_carter_docking``'s root directory.
It will set the robot's pose as virtually the dock's staging pose to bypass navigating to the staging pose and attempt docking immediately, then infinitely loop docking and undocking in a row.
This is a useful first-time setup to try docking, refine your detection offsets, and obtain reliability metrics of your complete system.
See the video below of this all at work!

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="708" height="400" src="https://www.youtube.com/embed/J3ygkehttlg?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
      </div>
    </h1>

Note that the robot is able to overcome:

- Large distances away from the dock staging pose, as long as the dock was in view
- Able to detect the dock's offsets and compute controls to dock successfully - including when we manually move it during and between runs
- Dock repeatedly with a 100% success rate due to the detections and charging state feedback

This script demonstrates the essential use of the Docking Server.
However, it does not use the dock database of pre-mapped dock locations that you setup. 
After you launch Nav2 and localize your robot in your map, we can adjust ``dockRobot()`` to take in your desired ``dock_id`` and perform docking instead:
Then, we can see the full docking system in action in a non-trivial environment!

.. code-block:: python

    def dockRobot(self, dock_id = ""):
        """Send a `DockRobot` action request."""
        print("Waiting for 'DockRobot' action server")
        while not self.docking_client.wait_for_server(timeout_sec=1.0):
            print('"DockRobot" action server not available, waiting...')

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = dock_id  # if wanting to use ID instead

        print('Docking at ID: ' + str(dock_id) + '...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg,
                                                                self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Docking request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    ...

    dock_id = 'home_dock'
    tester.dockRobot(dock_id)

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="708" height="400" src="https://www.youtube.com/embed/uHT5TeHJqZg?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
      </div>
    </h1>

Depending on your robot's relative pose to the dock and your pre-staging tolerance settings, Nav2 may attempt to navigate to the staging pose before docking.
If you wish to disable that, set ``goal_msg.navigate_to_staging_pose = False`` and then Docking will trigger immediately.
You can see both of these in action in the video above.

Don't want to call Docking Server from a script Python or C++ script and want to use it in your Autonomous Behavior Tree? See ``opennav_docking_bt`` for ``DockRobot``, ``UndockRobot`` Behavior Tree nodes to call the Docking Server from your application behavior tree -- with a provided ``XML`` example.
Note that if using ``navigate_to_staging_pose = True``, you cannot call ``DockRobot`` from inside a Nav2 Behavior Tree, only from your higher level autonomy tree since it recursively calls Nav2.
If you wish to call ``DockRobot`` from inside your Nav2 BT, you must roughly pre-stage the robot near the dock first (which should be easy as a navigation goal).
However, you can always call ``UndockRobot`` from any behavior tree!

Happy docking!
