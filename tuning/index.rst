.. _tuning:

Tuning Guide
############

This guide is meant to assist users in tuning their navigation system. While :ref:`configuration` is the home of the list of parameters for all of Nav2, it doesn't contain much *color* for how to tune a system using the most important of them.

This tuning guide is a perpetual work in progress. If you see some insights you have missing, please feel free to file a ticket or pull request with the wisdom you would like to share. This is an open section supported by the generosity of Nav2 users. Please consider paying it forward.

Inflation Potential Fields
==========================

Many users and default navigation configuration files the maintainers find are really missing the point of the inflation layer. While it's true that you can simply inflate a small radius around the walls to weight against critical collisions, the *true* value of the inflation layer is creating a consistent potential field around the entire map.

Some of the most popular tuning guides for ROS Navigation / Nav2 even [call this out specifically](https://arxiv.org/pdf/1706.09068.pdf) that there's substantial benefit to creating a gentle potential field across the width of the map - after inscribed costs are applied - yet very few users do this in practice.

This habit actually results in paths produced by NavFn, Theta\*, and Smac Planner to be somewhat suboptimal. They really want to look for a smooth potential field rather than wide open 0-cost spaces in order to stay in the middle of spaces and deal with close-by moving obstacles better. It will allow search to be weighted towards freespace far before the search algorithm runs into the obstacle that the inflation is caused by, letting the planner give obstacles as wide of a berth as possible.

So it is the maintainers' recommendation, as well as all other cost-aware search planners available in ROS, to increase your inflation layer cost scale in order to adequately produce a smooth potential across the entire map. For very large open spaces, its fine to have 0-cost areas in the middle, but for halls, aisles, and similar; **please create a smooth potential to provide the best performance**.

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


Other Pages We'd Love To Offer
==============================

If you are willing to chip in!

- Kinematic parameters for behaviors, controllers, etc
- Caching obstacle heuristic (Smac Hybrid-A*, State Lattice)
- AMCL
- Costmap 2D (local size + speed, update rate, downsampling sensors, resolution, unknown space, inflation params, obstacle/voxel params, etc)
- DWB (critics, generator, plugin)
- Goal checkers
- Progress checkers
- Controller selection
- Planner selection
- Costmap plugin selection
- Ideas in https://github.com/ros-planning/navigation.ros.org/issues/204
- ... anything you think would be insightful!
