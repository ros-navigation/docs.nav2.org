.. _writing_new_behavior_plugin:

Writing a New Behavior Plugin
*****************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

Overview
========

This tutorial shows how to create you own Behavior Plugin.
The Behavior Plugins live in the behavior server.
Unlike the planner and controller servers, each behavior will host its own unique action server.
The planners and controllers have the same API as they accomplish the same task.
However, recoveries can be used to do a wide variety of tasks, so each behavior can have its own unique action message definition and server.
This allows for massive flexibility in the behavior server enabling any behavior action imaginable that doesn't need to have other reuse.

Requirements
============

- ROS 2 (binary or build-from-source)
- Nav2 (Including dependencies)
- Gazebo
- Turtlebot3

Tutorial Steps
==============

1- Creating a new Behavior Plugin
---------------------------------

We will create a simple send sms behavior.
It will use Twilio to send a message via SMS to a remote operations center.
The code in this tutorial can be found in `navigation_tutorials <https://github.com/ros-planning/navigation2_tutorials>`_ repository as ``nav2_sms_behavior``.
This package can be a considered as a reference for writing Behavior Plugin.

Our example plugin implements the plugin class of ``nav2_core::Behavior``.
However, we have a nice wrapper for actions in ``nav2_behaviors``, so we use the ``nav2_behaviors::TimedBehavior`` base class for this application instead.
This wrapper class derives from the ``nav2_core`` class so it can be used as a plugin, but handles the vast majority of ROS 2 action server boiler plate required.

The base class from ``nav2_core`` provides 4 pure virtual methods to implement a Behavior Plugin.
The plugin will be used by the behavior server to host the plugins, but each plugin will provide their own unique action server interface.
Let's learn more about the methods needed to write a Behavior Plugin **if you did not use the ``nav2_behaviors`` wrapper**.

+----------------------+-----------------------------------------------------------------------------+-------------------------+
| **Virtual method**   | **Method description**                                                      | **Requires override?**  |
+----------------------+-----------------------------------------------------------------------------+-------------------------+
| configure()          | Method is called at when server enters on_configure state. Ideally          | Yes                     |
|                      | this method should perform declarations of ROS parameters and               |                         |
|                      | initialization of behavior's member variables. This method takes 4 input    |                         |
|                      | parameters: shared pointer to parent node, behavior name, tf buffer pointer |                         |
|                      | and shared pointer to a collision checker.                                  |                         |
+----------------------+-----------------------------------------------------------------------------+-------------------------+
| activate()           | Method is called when behavior server enters on_activate state. Ideally     | Yes                     |
|                      | this method should implement operations which are neccessary before the     |                         |
|                      | behavior goes to an active state.                                           |                         |
+----------------------+-----------------------------------------------------------------------------+-------------------------+
| deactivate()         | Method is called when behavior server enters on_deactivate state. Ideally   | Yes                     |
|                      | this method should implement operations which are neccessary before         |                         |
|                      | behavior goes to an inactive state.                                         |                         |
+----------------------+-----------------------------------------------------------------------------+-------------------------+
| cleanup()            | Method is called when behavior server goes to on_cleanup state. Ideally     | Yes                     |
|                      | this method should clean up resources which are created for the behavior.   |                         |
+----------------------+-----------------------------------------------------------------------------+-------------------------+

For the ``nav2_behaviors`` wrapper, which provides the ROS 2 action interface and boilerplate, we have 4 virtual methods to implement.
This tutorial uses this wrapper so these are the main elements we will address.

+----------------------+-----------------------------------------------------------------------------+-------------------------+
| **Virtual method**   | **Method description**                                                      | **Requires override?**  |
+----------------------+-----------------------------------------------------------------------------+-------------------------+
| onRun()              | Method is called immediately when a new behavior action request is          | Yes                     |
|                      | received. Gives the action goal to process and should start behavior        |                         |
|                      | initialization / process.                                                   |                         |
+----------------------+-----------------------------------------------------------------------------+-------------------------+
| onCycleUpdate()      | Method is called at the behavior update rate and should complete any        | Yes                     |
|                      | necessary updates. An example for spinning is computing the command         |                         |
|                      | velocity for the current cycle, publishing it and checking for completion.  |                         |
+----------------------+-----------------------------------------------------------------------------+-------------------------+
| onConfigure()        | Method is called when behavior server enters on_configure state. Ideally    | No                      |
|                      | this method should implement operations which are neccessary before         |                         |
|                      | behavior goes to a configured state (get parameters, etc).                  |                         |
+----------------------+-----------------------------------------------------------------------------+-------------------------+
| onCleanup()          | Method is called when behavior server goes to on_cleanup state. Ideally     | No                      |
|                      | this method should clean up resources which are created for the behavior.   |                         |
+----------------------+-----------------------------------------------------------------------------+-------------------------+

For this tutorial, we will be using methods ``onRun()``, ``onCycleUpdate()``, and ``onConfigure()`` to create the SMS behavior.
``onConfigure()`` will be skipped for brevity, but only declares parameters.

In recoveries, ``onRun()`` method must set any initial state and kick off the behavior.
For the case of our call for help behavior, we can trivially compute all of our needs in this method.

.. code-block:: c++

  Status SendSms::onRun(const std::shared_ptr<const Action::Goal> command)
  {
    std::string response;
    bool message_success = _twilio->send_message(
      _to_number,
      _from_number,
      command->message,
      response,
      "",
      false);

    if (!message_success) {
      RCLCPP_INFO(node_->get_logger(), "SMS send failed.");
      return Status::FAILED;
    }

    RCLCPP_INFO(node_->get_logger(), "SMS sent successfully!");
    return Status::SUCCEEDED;
  }

We receive an action goal, ``command``, which we want to process.
``command`` contains a field ``message`` that contains the message we want to communicate to our mothership.
This is the "call for help" message that we want to send via SMS to our brothers in arms in the operations center.

We use the service Twilio to complete this task.
Please `create an account <https://www.twilio.com/>`_ and get all the relavent information needed for creating the service (e.g. ``account_sid``, ``auth_token``, and a phone number).
You can set these values as parameters in your configuration files corresponding to the ``onConfigure()`` parameter declarations.

We use the ``_twilio`` object to send our message with your account information from the configuration file.
We send the message and log to screen whether or not the message was sent successfully or not.
We return a ``FAILED`` or ``SUCCEEDED`` depending on this value to be returned to the action client.

``onCycleUpdate()`` is trivially simple as a result of our short-running behavior.
If the behavior was instead longer running like spinning, navigating to a safe area, or getting out of a bad spot and waiting for help, then this function would be checking for timeouts or computing control values.
For our example, we simply return success because we already completed our mission in ``onRun()``.

.. code-block:: c++

  Status SendSms::onCycleUpdate()
  {
    return Status::SUCCEEDED;
  }

The remaining methods are not used and are not mandatory to override them.

2- Exporting the Behavior Plugin
--------------------------------

Now that we have created our custom behavior, we need to export our Behavior Plugin so that it would be visible to the behavior server. Plugins are loaded at runtime and if they are not visible, then our behavior server won't be able to load it. In ROS 2, exporting and loading plugins is handled by ``pluginlib``.

Coming to our tutorial, class ``nav2_sms_bahavior::SendSms`` is loaded dynamically as ``nav2_core::Behavior`` which is our base class.

1. To export the behavior, we need to provide two lines

.. code-block:: c++

  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(nav2_sms_bahavior::SendSms, nav2_core::Behavior)

Note that it requires pluginlib to export out plugin's class. Pluginlib would provide as macro ``PLUGINLIB_EXPORT_CLASS`` which does all the work of exporting.

It is good practice to place these lines at the end of the file but technically, you can also write at the top.

2. Next step would be to create plugin's description file in the root directory of the package. For example, ``behavior_plugin.xml`` file in our tutorial package. This file contains following information

 - ``library path``: Plugin's library name and it's location.
 - ``class name``: Name of the class.
 - ``class type``: Type of class.
 - ``base class``: Name of the base class.
 - ``description``: Description of the plugin.

.. code-block:: xml

  <library path="nav2_sms_behavior_plugin">
    <class name="nav2_sms_behavior/SendSms" type="nav2_sms_behavior::SendSms" base_class_type="nav2_core::Behavior">
      <description>This is an example plugin which produces an SMS text message recovery.</description>
    </class>
  </library>

3. Next step would be to export plugin using ``CMakeLists.txt`` by using cmake function ``pluginlib_export_plugin_description_file()``. This function installs plugin description file to ``share`` directory and sets ament indexes to make it discoverable.

.. code-block:: text

  pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

4. Plugin description file should also be added to ``package.xml``

.. code-block:: xml

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/behavior_plugin.xml" />
  </export>

5. Compile and it should be registered. Next, we'll use this plugin.


3- Pass the plugin name through params file
-------------------------------------------

To enable the plugin, we need to modify the ``nav2_params.yaml`` file as below to replace following params

.. code-block:: yaml

  behavior_server:  # Humble and later
  recoveries_server:  # Galactic and earlier
    ros__parameters:
      costmap_topic: local_costmap/costmap_raw
      footprint_topic: local_costmap/published_footprint
      cycle_frequency: 10.0
      behavior_plugins: ["spin", "backup", "wait"]  # Humble and later
      recovery_plugins: ["spin", "backup", "wait"]  # Galactic and earlier
      spin:
        plugin: "nav2_behaviors/Spin"
      backup:
        plugin: "nav2_behaviors/BackUp"
      wait:
        plugin: "nav2_behaviors/Wait"
      global_frame: odom
      robot_base_frame: base_link
      transform_timeout: 0.1
      use_sim_time: true
      simulate_ahead_time: 2.0
      max_rotational_vel: 1.0
      min_rotational_vel: 0.4
      rotational_acc_lim: 3.2

with

.. code-block:: yaml

  behavior_server:  # Humble and newer
  recoveries_server:  # Galactic and earlier
    ros__parameters:
      local_costmap_topic: local_costmap/costmap_raw
      local_footprint_topic: local_costmap/published_footprint
      global_costmap_topic: global_costmap/costmap_raw
      global_footprint_topic: global_costmap/published_footprint
      cycle_frequency: 10.0
      behavior_plugins: ["spin", "backup", "wait","send_sms"]  # Humble and newer
      recovery_plugins: ["spin", "backup", "wait","send_sms"]  # Galactic and earlier
      spin:
        plugin: "nav2_behaviors/Spin"
      backup:
        plugin: "nav2_behaviors/BackUp"
      wait:
        plugin: "nav2_behaviors/Wait"
      send_sms:
        plugin: "nav2_sms_behavior/SendSms"
      account_sid: ... # your sid
      auth_token: ... # your token
      from_number: ... # your number
      to_number: ... # the operations center number
      global_frame: odom
      robot_base_frame: base_link
      transform_timeout: 0.1
      use_sim_time: true
      simulate_ahead_time: 2.0
      max_rotational_vel: 1.0
      min_rotational_vel: 0.4
      rotational_acc_lim: 3.2

In the above snippet, you can observe that we add the SMS behavior under the ``send_sms`` ROS 2 action server name.
We also tell the behavior server that the ``send_sms`` is of type ``SendSms`` and give it our parameters for your Twilio account.

4- Run Behavior Plugin
----------------------

Run Turtlebot3 simulation with enabled Nav2. Detailed instruction how to make it are written at :ref:`getting_started`. Below is shortcut command for that:

.. code-block:: bash

  $ ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml

In a new terminal run:

.. code-block:: bash

  $ ros2 action send_goal "send_sms" nav2_sms_behavior/action/SendSms "{message : Hello!! Navigation2 World }"
