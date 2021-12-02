.. _tuning:

Tuning Guide
############

This guide is meant to assist users in tuning their navigation system. While :ref:`configuration` is the home of the list of parameters for all of Nav2, it doesn't contain much *color* for how to tune a system using the most important of them. The aim of this guide is to give more advice in how to setup your system beyond a first time setup, which you can find at :ref:`setup_guides`. This will by no means cover all of the parameters (so please, do review the configuration guides for the packages of interest), but will give some helpful hints and tips.

This tuning guide is a perpetual work in progress. If you see some insights you have missing, please feel free to file a ticket or pull request with the wisdom you would like to share. This is an open section supported by the generosity of Nav2 users and maintainers. Please consider paying it forward.

Inflation Potential Fields
==========================

Many users and ecosystem navigation configuration files the maintainers find are really missing the point of the inflation layer. While it's true that you can simply inflate a small radius around the walls to weight against critical collisions, the *true* value of the inflation layer is creating a consistent potential field around the entire map.

Some of the most popular tuning guides for ROS Navigation / Nav2 even `call this out specifically <https://arxiv.org/pdf/1706.09068.pdf>`_ that there's substantial benefit to creating a gentle potential field across the width of the map - after inscribed costs are applied - yet very few users do this in practice.

This habit actually results in paths produced by NavFn, Theta\*, and Smac Planner to be somewhat suboptimal. They really want to look for a smooth potential field rather than wide open 0-cost spaces in order to stay in the middle of spaces and deal with close-by moving obstacles better. It will allow search to be weighted towards freespace far before the search algorithm runs into the obstacle that the inflation is caused by, letting the planner give obstacles as wide of a berth as possible.

So it is the maintainers' recommendation, as well as all other cost-aware search planners available in ROS, to increase your inflation layer cost scale and radius in order to adequately produce a smooth potential across the entire map. For very large open spaces, its fine to have 0-cost areas in the middle, but for halls, aisles, and similar; **please create a smooth potential to provide the best performance**.

Robot Footprint vs Radius
=========================

Nav2 allows users to specify the robot's shape in 2 ways: a geometric ``footprint`` or the radius of a circle encompassing the robot. In ROS (1), it was pretty reasonable to always specify a radius approximation of the robot, since the global planning algorithms didn't use it and the local planners / costmaps were set up with the circular assumption baked in.

However, in Nav2, we now have multiple planning and controller algorithms that make use of the full SE2 footprint. If your robot is non-circular, it is recommended that you give the planners and controllers the actual, geometric footprint of your robot. This will allow the planners and controllers to plan or create trajectories into tighter spaces. For example, if you have a very long but skinny robot, the circular assumption wouldn't allow a robot to plan into a space only a little wider than your robot, since the robot would not fit length-wise.

The kinematically feasible planners (e.g. Smac Hybrid-A\*, Smac State Lattice) will use the SE2 footprint for collision checking to create kinematically feasible plans, if provided with the actual footprint. As of December, 2021 all of the controller plugins support full footprint collision checking to ensure safe path tracking. If you provide a footprint of your robot, it will be used to make sure trajectories are valid and it is recommended you do so. It will prevent a number of "stuck robot" situations that could have been easily avoided.

If your robot is truly circular, continue to use the ``robot_radius`` parameter. The three valid reasons for a non-circular robot to use the radius instead:

- The robot is very small relative to the environment (e.g. RC car in a warehouse)
- The robot has such limited compute power, using SE2 footprint checking would add too much computational burden (e.g. embedded micro-processors)
- If you plan to use a holonomic planner (e.g. Theta\*, Smac 2D-A\*, or NavFn), you may continue to use the circular footprint, since these planners are not kinematically feasible and will not make use of the SE2 footprint anyway.


Rotate in Place Behavior
========================

Using the :ref:`configuring_rotation_shim`, a robot will simply rotate in place before starting to track a holonomic path. This allows a developer to tune a controller plugin to be optimized for path tracking and give you clean rotations, out of the box.

This was added due to quirks in some existing controllers whereas tuning the controller for a task can make it rigid -- or the algorithm simply doesn't rotate in place when working with holonomic paths (if that's a desirable trait). The result is an awkward, stuttering, or whipping around behavior when your robot's initial and path heading's are significantly divergent. Giving a controller a better starting point to start tracking a path makes tuning the controllers significantly easier and creates more intuitive results for on-lookers (in one maintainer's opinion).

Note: If using a non-holonomic, kinematically feasible planner (e.g. Smac Hybrid-A\*, Smac State Lattice), this is not a necessary behavioral optimization. This class of planner will create plans that take into account the robot's starting heading, not requiring any rotation behaviors. 

This behavior is most optimially for: 

- Robots that can rotate in place, such as differential and omnidirectional robots.
- Preference to rotate in place when starting to track a new path that is at a significantly different heading than the robot’s current heading – or when tuning your controller for its task makes tight rotations difficult.

Planner Plugin Selection
========================

Nav2 provides a number of planning plugins out of the box. For a first-time setup, see :ref:`select_algorithm` for a more verbose breakdown of algorithm styles within Nav2, and :ref:`plugins` for a full accounting of the current list of plugins available (which may be updated over time).

In general though, the following table is a good guide for the optimal planning plugin for different types of robot bases:

+------------------------+----------------------------------------------------------------------+
| Plugin Name            | Supported Robot Types                                                |
+========================+======================================================================+
| NavFn Planner          | Circular Differential, Circular Omnidirectional                      |   
+------------------------+                                                                      |
| Smac Planner 2D        |                                                                      |
+------------------------+                                                                      |
| Theta Star Planner     |                                                                      |
+------------------------+----------------------------------------------------------------------+
| Smac Hybrid-A* Planner | Non-circular or Circular Ackermann, Non-circular or Circular Legged  |
+------------------------+----------------------------------------------------------------------+
| Smac Lattice Planner   | Non-circular Differential, Non-circular Omnidirectional, Arbitrary   |
+------------------------+----------------------------------------------------------------------+

If you are using a non-circular robot with very limited compute, it may be worth assessing the benefits of using one of the holonomic planners (e.g. particle assumption planners). It is the recommendation of the maintainers to start using one of the more advanced algorithms appropriate for your platform *first*, but to scale back the planner if need be. The run-time of the feasible planners are typically on par (or sometimes faster) than their holonomic counterparts, so don't let the more recent nature of them fool you.

Since the planning problem is primarily driven by the robot type, the table accurately summarizes the advice to users by the maintainers.

.. note::
   These are simply the default and available plugins from the community. For a specific application / platform, you may also choose to use none of these and create your own, and that's the intention of the Nav2 framework. See the :ref:`writing_new_nav2planner_plugin` tutorial for more details. If you're willing to contribute this work back to the community, please file a ticket or contact a maintainer! They'd love to hear from you.

Controller Plugin Selection
===========================

Nav2 provides a number of controller plugins out of the box. For a first-time setup, see :ref:`select_algorithm` for a more verbose breakdown of algorithm styles within Nav2, and :ref:`plugins` for a full accounting of the current list of plugins available (which may be updated over time).

In general though, the following table is a good first-order description of the controller plugins available for different types of robot bases:

+----------------+---------------------------------------------------+----------------------------+
| Plugin Name    | Supported Robot Types                             | Task                       |
+================+===================================================+============================+
| DWB controller | Differential, Omnidirectional                     | Dynamic obstacle avoidance |
+----------------+---------------------------------------------------+                            |
| TEB Controller | Differential, Omnidirectional, Ackermann, Legged  | Dynamic obstacle avoidance |
+----------------+---------------------------------------------------+----------------------------+
| RPP controller | Differential, Ackermann, Legged                   | Exact path following       |
+----------------+---------------------------------------------------+----------------------------+
| Rotation Shim  | Differential, Omnidirectional                     | Rotate to rough heading    |


All of the above controllers can handle both circular and arbitrary shaped robots in configuration.

Regulated Pure Pursuit is good for exact path following and is typically paired with one of the kinematically feasible planners (eg State Lattice, Hybrid-A\*, etc) since those paths are known to be drivable given hard physical constraints. However, it can also be applied to differential drive robots who can easily pivot to match any holonomic path. This is the plugin of choice if you simply want your robot to follow the path, rather exactly, without any dynamic obstacle avoidance or deviation. It is simple and geometric, as well as slowing the robot in the presence of near-by obstacles *and* while making sharp turns.

DWB and TEB are both options that will track paths, but also diverge from the path if there are dynamic obstacles present (in order to avoid them). DWB does this through scoring multiple trajectories on a set of critics. These critics are plugins that can be selected at run-time and contain weights that may be tuned to create the desired behavior. This does require a bit of tuning for a given platform, application, and desired behavior. 

TEB on the other hand implements an optimization based approach, generating a graph-solving problem for path tracking in the presense of obstacles. This typically works pretty well out of the box, but to tune for specific behaviors, you may have to modify optimization engine parameters which are not as intuitive or rooted in something physical as DWB. 

Finally, the Rotation Shim Plugin helps assist plugins like TEB and DWB (among others) to rotate the robot in place towards a new path's heading before starting to track the path. This allows you to tune your local trajectory planner to operate with a desired behavior without having to worry about being able to rotate on a dime with a significant deviation in angular distance over a very small euclidean distance. Some controllers when heavily tuned for accurate path tracking are constrained in their actions and don't very cleanly rotate to a new heading. This helps alleviate that problem and makes the robot rotate in place very smoothly.

.. note::
   These are simply the default and available plugins from the community. For a specific robot platform / company, you may also choose to use none of these and create your own. See the :ref:`writing_new_nav2controller_plugin` tutorial for more details. If you're willing to contribute this work back to the community, please file a ticket or contact a maintainer! They'd love to hear from you.


Other Pages We'd Love To Offer
==============================

TODO update maintainers list to include tuning guide updates

If you are willing to chip in!

- Kinematic parameters for behaviors, controllers, etc
- Caching obstacle heuristic (Smac Hybrid-A*, State Lattice)

- AMCL / weights
- Costmap 2D (local size + speed, update rate, downsampling sensors, resolution, unknown space/inflate, obstacle/voxel params, etc)
- DWB (critics, generator, plugin, weights)
- Goal checkers
- Progress checkers
- recovery speeds / sim times
- Costmap plugin selection
- Ideas in https://github.com/ros-planning/navigation.ros.org/issues/204
- ... anything you think would be insightful!
