.. _shim_tutorial:

Using Rotation Shim Controller
******************************


- `Overview`_ 
- `What is the Rotation Shim Controller?`_
- `Configuring Rotation Shim Controller`_
- `Configuring Primary Controller`_
- `Demo Execution`_

Overview
========

This tutorial will discuss how to set up your robot to use the ``RotationShimController`` to help create intuitive, rotate-in-place, behavior for your robot while starting out to track a path. The goal of this tutorial is to explain to the reader the value of the controller, how to configure it, how to configure the primary controller with it, and finally an example of it in use.

Before starting this tutorial, completing the :ref:`getting_started` is highly recommended especially if you are new to ROS and Nav2. The requirements are having the latest install of Nav2 / ROS 2 containing this package.

What is the Rotation Shim Controller?
=====================================

This was developed due to quirks in TEB and DWB, but applicable to any other controller plugin type that you'd like to have rotation in place behavior with. ``TEB``'s behavior tends to whip the robot around with small turns, or when the path is starting at a very different heading than current, in a somewhat surprising way due to the elastic band approach. ``DWB`` can be tuned to have any type of behavior, but typically to tune it to be an excellent path follower also makes it less optimally capable of smooth transitions to new paths at far away headings -- there are always trade offs. Giving both TEB and DWB a better starting point to start tracking a path makes tuning the controllers significantly easier and creates more intuitive results for on-lookers. 

Note that it is not required to use this with **any** plugin. Many users are perfectly successful without using this controller, but if a robot may rotate in place before beginning its path tracking task (or others), it can be advantageous to do so. 

The ``nav2_rotation_shim_controller`` will check the rough heading difference with respect to the robot and a newly received path. If within a threshold, it will pass the request onto the ``primary_controller`` to execute the task. If it is outside of the threshold, this controller will rotate the robot in place towards that path heading. Once it is within the tolerance, it will then pass off control-execution from this rotation shim controller onto the primary controller plugin. At this point, the robot's main plugin will take control for a smooth hand off into the task. 

The ``RotationShimController`` is most suitable for:

- Robots that can rotate in place, such as differential and omnidirectional robots.
- Preference to rotate in place when starting to track a new path that is at a significantly different heading than the robot's current heading -- or when tuning your controller for its task makes tight rotations difficult.
- Using planners that are non-kinematically feasible, such as NavFn, Theta\*, or Smac 2D (Feasible planners such as Smac Hybrid-A* and State Lattice will start search from the robot's actual starting heading, requiring no rotation since their paths are guaranteed drivable by physical constraints). 

.. note::
  Regulated Pure Pursuit has this built in so it is not necessary to pair with RPP. However, it is applicable to all others. See :ref:`plugins` for a full list of current controller plugins.

Configuring Rotation Shim Controller
====================================

This controller is a *shim* because it is placed between the primary controller plugin and the controller server. It takes commands and pre-processes them to rotate to the heading and then passes off execution-control to the primary plugin once that condition is met - acting as a simple pass through.

As such, its configuration looks very similar to that of any other plugin. In the code block below, you can see that we've added the ``RotationShimController`` as the plugin for path tracking in the controller server. You can see that we've also configured it below with its internal parameters, ``angular_dist_threshold`` through ``max_angular_accel``.

.. code-block:: yaml

    controller_server:
      ros__parameters:
        use_sim_time: True
        controller_frequency: 20.0
        min_x_velocity_threshold: 0.001
        min_y_velocity_threshold: 0.5
        min_theta_velocity_threshold: 0.001
        progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
        goal_checker_plugins: ["goal_checker"]
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
          plugin: "nav2_rotation_shim_controller::RotationShimController"
          angular_dist_threshold: 0.785
          forward_sampling_distance: 0.5
          rotate_to_heading_angular_vel: 1.8
          max_angular_accel: 3.2
          simulate_ahead_time: 1.0

The Rotation Shim Controller is very simple and only has a couple of parameters to dictate the conditions it should be enacted.

- ``angular_dist_threshold``: The angular distance (in radians) apart from the robot's current heading and the approximated path heading to trigger the rotation behavior. Once the robot is within this threshold, control is handed over to the primary controller plugin.
- ``forward_sampling_distance``: The distance (in meters) away from the robot to select a point on the path to approximate the path's starting heading at. This is analogous to a "lookahead" point.
- ``rotate_to_heading_angular_vel``: The angular velocity (in rad/s) to have the robot rotate to heading by, when the behavior is enacted.
- ``max_angular_accel``: The angular acceleration (in rad/s/s) to have the robot rotate to heading by, when the behavior is enacted.
- ``simulate_ahead_time``: The Time (s) to forward project the rotation command to check for collision
                
Configuring Primary Controller
==============================

There is one more remaining parameter of the ``RotationShimController`` not mentioned above, the ``primary_controller``. This is the type of controller that your application would like to use as the primary modus operandi. It will share the same name and yaml namespace as the shim plugin. You can observe this below with the primary controller set the ``DWB`` (with the progress and goal checkers removed for brevity). 

.. code-block:: yaml

    controller_server:
      ros__parameters:
        use_sim_time: True
        controller_frequency: 20.0
        min_x_velocity_threshold: 0.001
        min_y_velocity_threshold: 0.5
        min_theta_velocity_threshold: 0.001
        controller_plugins: ["FollowPath"]
        FollowPath:
          plugin: "nav2_rotation_shim_controller::RotationShimController"
          primary_controller: "dwb_core::DWBLocalPlanner"
          angular_dist_threshold: 0.785
          forward_sampling_distance: 0.5
          rotate_to_heading_angular_vel: 1.8
          max_angular_accel: 3.2
          simulate_ahead_time: 1.0

          # DWB parameters
          ...
          ...
          ...

An important note is that **within the same yaml namespace**, you may also include any ``primary_controller`` specific parameters required for a robot. Thusly, after ``max_angular_accel``, you can include any of ``DWB``'s parameters for your platform. 


Demo Execution
==============

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="708" height="400" src="https://www.youtube.com/embed/t-g2CBGByEw?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
      </div>
    </h1>
