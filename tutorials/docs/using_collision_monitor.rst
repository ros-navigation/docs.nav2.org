.. _collision_monitor_tutorial:

Using Collision Monitor
***********************

- `Overview`_
- `Requirements`_
- `Preparing Nav2 stack`_
- `Configuring Collision Monitor`_
- `Demo Execution`_

.. image:: images/Collision_Monitor/collision_monitor.gif
  :width: 800px

Overview
========

This tutorial shows how to use a Collision Monitor with Nav2 stack. Based on this tutorial, you can setup it for your environment and needs.

Requirements
============

It is assumed ROS2 and Nav2 dependent packages are installed or built locally.
Please make sure that Nav2 project is also built locally as it was made in :ref:`build-instructions`.

Configuring Collision Monitor
=============================

The Collision Monitor node has its own ``collision_monitor_node.launch.py`` launch-file and preset parameters in the ``collision_monitor_params.yaml`` file.
For the demonstration, two shapes will be created - an inner circle stop area around the robot and a larger slowdown bounding box:

.. image:: images/Collision_Monitor/polygons.png
  :width: 800px

If more than 3 points will appear inside a slowdown box, the robot will decrease its speed to ``30%`` from its value, allowing more room to avoid obstacles.
For the cases when obstacles are dangerously close to the robot, inner stop circle will work.
For this setup, the following lines should be added into ``collision_monitor_params.yaml`` parameters file. Stop circle is named as ``CircleStop`` and slowdown bounding box - as ``PolygonSlow``:

.. code-block:: yaml

    polygons: ["CircleStop", "PolygonSlow"]
      CircleStop:
        type: "circle"
        radius: 0.3
        aaction_type: "stop"
        max_points: 3
        visualize: True
        polygon_pub_topic: "polygon_stop"
      PolygonSlow:
        type: "polygon"
        points: [0.4, 0.4, 0.4, -0.4, -0.4, -0.4, -0.4, 0.4]
        action_type: "slowdown"
        max_points: 3
        slowdown_ratio: 0.3
        visualize: True
        polygon_pub_topic: "polygon_slowdown"

For the working configuration, at least one data source should be added.
In current demonstration, it is used laser scanner, which is described by the following lines for Collision Monitor node:

.. code-block:: yaml

    observation_sources: ["scan"]
      scan:
        type: "scan"
        topic: "/scan"

Set topic names, frame ID-s and timeouts to work correctly with a default Nav2 setup.
The whole ``nav2_collision_monitor/params/collision_monitor_params.yaml`` file in this case will look as follows:

.. code-block:: yaml

    collision_monitor:
      ros__parameters:
        use_sim_time: True
        base_frame_id: "base_footprint"
        odom_frame_id: "odom"
        cmd_vel_in_topic: "cmd_vel_raw"
        cmd_vel_out_topic: "cmd_vel"
        transform_tolerance: 0.5
        source_timeout: 5.0
        stop_pub_timeout: 2.0
        polygons: ["CircleStop", "PolygonSlow"]
        CircleStop:
          type: "circle"
          radius: 0.3
          action_type: "stop"
          max_points: 3
          visualize: True
          polygon_pub_topic: "polygon_stop"
        PolygonSlow:
          type: "polygon"
          points: [0.4, 0.4, 0.4, -0.4, -0.4, -0.4, -0.4, 0.4]
          action_type: "slowdown"
          max_points: 3
          slowdown_ratio: 0.3
          visualize: True
          polygon_pub_topic: "polygon_slowdown"
        observation_sources: ["scan"]
        scan:
          type: "scan"
          topic: "/scan"

Preparing Nav2 stack
====================

Since Collision Monitor is designed to operate as an independent safety node, Nav2 stack has no knowledge about it.
It is laying under the stack and operating after all necessary decisions were made by Nav2.
This is achieved through remapped ``cmd_vel`` topic, going out from a Controller.
This is being made by means of adding the remapping as written below to the ``navigation_launch.py`` bringup script.
Please note, that remapped ``cmd_vel_raw`` topic should match to the ``cmd_vel_in_topic`` parameter value of Collision Monitor node, and ``cmd_vel_out_topic`` parameter value should be equal to initial ``cmd_vel`` to fit the replacement:

.. code-block:: python

         remappings = [('/tf', 'tf'),
    -                  ('/tf_static', 'tf_static')]
    +                  ('/tf_static', 'tf_static'),
    +                  ('/cmd_vel', '/cmd_vel_raw')]

Since Collision Monitor performs effectively to avoid collisions, we need to increase the probability of it, e.g. by allowing the robot to get closer to the obstacles.
For that, let's almost remove the inflation radius around obstacles from ``0.55`` to ``0.02`` meters in a ``nav2_params.yaml`` Nav2 bringup parameters for both local and global costmaps:

.. code-block:: yaml

    local_costmap:
      local_costmap:
        ros__parameters:
          ...
          inflation_layer:
            plugin: "nav2_costmap_2d::InflationLayer"
            cost_scaling_factor: 3.0
    -       inflation_radius: 0.55
    +       inflation_radius: 0.02
    ...
    global_costmap:
      global_costmap:
        ros__parameters:
          ...
          inflation_layer:
            plugin: "nav2_costmap_2d::InflationLayer"
            cost_scaling_factor: 3.0
    -       inflation_radius: 0.55
    +       inflation_radius: 0.02

Demo Execution
==============

Once Collision Monitor node has been tuned, ``cmd_vel`` topics remapped and obstacle avoidance decreased, Collision Monitor node is ready to run.
For that, run Nav2 stack as written in :ref:`getting_started`:

.. code-block:: bash

  ros2 launch nav2_bringup tb3_simulation_launch.py

In parallel console, launch Collision Monitor node by using its launch-file:

.. code-block:: bash

  ros2 launch nav2_collision_monitor collision_monitor_node.launch.py

Since both ``CircleStop`` and ``PolygonSlow`` polygons will have their own publishers, they could be added to visualization as shown at the picture below:

.. image:: images/Collision_Monitor/polygons_visualization.png
  :width: 800px

Set the initial pose and then put Nav2 goal on map.
The robot will start its movement, slowing down while running near the obstacles, and stopping in close proximity to them:

.. image:: images/Collision_Monitor/collision.png
  :width: 800px
