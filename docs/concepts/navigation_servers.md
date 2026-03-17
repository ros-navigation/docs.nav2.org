# Navigation Servers

Planners and controllers are at the heart of a navigation task.
Recoveries are used to get the robot out of a bad situation or attempt to deal with various forms of issues to make the system fault-tolerant.
Smoothers can be used for additional quality improvements of the planned path.
In this section, the general concepts around them and their uses in this project are analyzed.

## Planner, Controller, Smoother, Route, and Behavior Servers

Five of the action servers in this project are the planner, behavior, smoother, route, and controller servers.

These action servers are used to host a map of algorithm plugins to complete various tasks.
They also host the environmental representation used by the algorithm plugins to compute their outputs.

The planner, smoother and controller servers will be configured at runtime with the names (aliases) and types of algorithms to use.
These types are the pluginlib names that have been registered and the names are the aliases for the task.
An example would be the DWB controller used with name `FollowPath`, as it follows a reference path.
In this case, then all parameters for DWB would be placed in that namespace, e.g. `FollowPath.<param>`.

These three servers then expose an action interface corresponding to their task.
When the behavior tree ticks the corresponding BT node, it will call the action server to process its task.
The action server callback inside the server will call the chosen algorithm by its name (e.g. `FollowPath`) that maps to a specific algorithm.
This allows a user to abstract the algorithm used in the behavior tree to classes of algorithms.
For instance, you can have `N` plugin controllers to follow paths, dock with charger, avoid dynamic obstacles, or interface with a tool.
Having all of these plugins in the same server allows the user to make use of a single environmental representation object, which is costly to duplicate.

For the behavior server, each of the behaviors also contains their own name, however, each plugin will also expose its own special action server.
This is done because of the wide variety of behavior actions that may be created which cannot have a single simple interface to share.
The behavior server also contains a costmap subscriber to the local costmap, receiving real-time updates from the controller server, to compute its tasks.
We do this to avoid having multiple instances of the local costmap which are computationally expensive to duplicate.

The route server does not contain multiple “routing algorithms” like the planner or controller servers.
Instead, it computes a route using a navigation graph using a set of plugins for scoring edges in the graph, parsing graph files, and performing operations along the route, if necessary.
Rather than freespace planning, this computes a route using a graph that can be generated to represent lanes, areas the robot is allowed to navigate, a teach-and-repeat route, urban roadways, and more.

Alternatively, since the BT nodes are trivial plugins calling an action, new BT nodes can be created to call other action servers with other action types.
It is advisable to use the provided servers if possible at all times.
If, due to the plugin or action interfaces, a new server is needed, that can be sustained with the framework.
The new server should use the new type and plugin interface, similar to the provided servers.
A new BT node plugin will need to be created to call the new action server – however no forking or modification is required in the Nav2 repo itself by making extensive use of servers and plugins.

If you find that you require a new interface to the pluginlib definition or action type, please file a ticket and see if we can rectify that in the same interfaces.

## Planners

The task of a planner is to compute a path to complete some objective function.
The path can also be known as a route, depending on the nomenclature and algorithm selected.
Two canonical examples are computing a plan to a goal (e.g. from current position to a goal) or complete coverage (e.g. plan to cover all free space).
The planner will have access to a global environmental representation and sensor data buffered into it.
Planners can be written to:

- Compute shortest path
- Compute complete coverage path
- Compute paths along sparse or predefined routes

The general task in Nav2 for the planner is to compute a valid, and potentially optimal, path from the current pose to a goal pose.
However, many classes of plans and routes exist which are supported.

## Controllers

Controllers, also known as local planners in ROS 1, are the way we follow the globally computed path or complete a local task.
The controller will have access to a local environment representation to attempt to compute feasible control efforts for the base to follow.
Many controller will project the robot forward in space and compute a locally feasible path at each update iteration.
Controllers can be written to:

- Follow a path
- Dock with a charging station using detectors in the odometric frame
- Board an elevator
- Interface with a tool

The general task in Nav2 for a controller is to compute a valid control effort to follow the global plan.
However, many classes of controllers and local planners exist.
It is the goal of this project that all controller algorithms can be plugins in this server for common research and industrial tasks.

## Behaviors

Recovery behaviors are a mainstay of fault-tolerant systems.
The goal of recoveries are to deal with unknown or failure conditions of the system and autonomously handle them.
Examples may include faults in the perception system resulting in the environmental representation being full of fake obstacles.
The clear costmap recovery would then be triggered to allow the robot to move.

Another example would be if the robot was stuck due to dynamic obstacles or poor control.
Backing up or spinning in place, if permissible, allow the robot to move from a poor location into free space it may navigate successfully.

Finally, in the case of a total failure, a recovery may be implemented to call an operator’s attention for help.
This can be done via email, SMS, Slack, Matrix, etc.

It is important to note that the behavior server can hold any behavior to share access to expensive resources like costmaps or TF buffers, not just recovery behaviors. Each may have its own API.

## Smoothers

As criteria for optimality of the path searched by a planner are usually reduced compared to reality, additional path refinement is often beneficial.
Smoothers have been introduced for this purpose, typically responsible for reducing path raggedness and smoothing abrupt rotations,
but also for increasing distance from obstacles and high-cost areas as the smoothers have access to a global environmental representation.

Use of a separate smoother over one that is included as part of a planner is advantageous when combining different planners with different smoothers or when a specific control over smoothing is required, e.g. smoothing only a specific part of the path.

The general task in Nav2 for a smoother is to receive a path and return its improved version.
However, for different input paths, criteria of the improvements and methods of acquiring them exist, creating space for a multitude of smoothers that can be registered in this server.

## Route

The route server is a specialized planner that computes a route using a navigation graph, rather than the freespace costmap.
The route is computed as the optimal way from the start to the goal through the set of nodes and directional edges in the pre-defined navigation graph.
This navigation graph can be generated to represent lanes, areas the robot is allowed to navigate, a teach-and-repeat route, urban roadways, and more.

## Robot Footprints

It is worth remarking that in the cost maps, we set a robot’s footprint either as a circle of radius `robot_radius` or as a vector of points `footprint` representing an arbitrary polygon if the robot is non-circular. This can also be adjusted over time using the costmap’s `~/footprint` topic, which will update the polygon over time as needed due to changes in the robot’s state, such as movement of an attached manipulator, picking up a pallet, or other actions that adjust a robot’s shape. That polygon will then automatically be used by the planners and controllers.

## Waypoint Following

Waypoint following is a basic feature of a navigation system. It tells our system how to use navigation to get to multiple destinations.

The `nav2_waypoint_follower` contains a waypoint following program with a plugin interface for specific task executors.
This is useful if you need to go to a given location and complete a specific task like take a picture, pick up a box, or wait for user input.
It is a nice demo application for how to use Nav2 in a sample application.

However, it could be used for more than just a sample application.
There are 2 schools of thoughts for fleet managers / dispatchers:

- Dumb robot; smart centralized dispatcher
- Smart robot; dumb centralized dispatcher

In the first, the `nav2_waypoint_follower` is fully sufficient to create a production-grade on-robot solution. Since the autonomy system / dispatcher is taking into account things like the robot’s pose, battery level, current task, and more when assigning tasks, the application on the robot just needs to worry about the task at hand and not the other complexities of the system to complete the requested task. In this situation, you should think of a request to the waypoint follower as 1 unit of work (e.g. 1 pick in a warehouse, 1 security patrole loop, 1 aisle, etc) to do a task and then return to the dispatcher for the next task or request to recharge. In this school of thought, the waypoint following application is just one step above navigation and below the system autonomy application.

In the second, the `nav2_waypoint_follower` is a nice sample application / proof of concept, but you really need your waypoint following / autonomy system on the robot to carry more weight in making a robust solution. In this case, you should use the `nav2_behavior_tree` package to create a custom application-level behavior tree using navigation to complete the task. This can include subtrees like checking for the charge status mid-task for returning to dock or handling more than 1 unit of work in a more complex task. Soon, there will be a `nav2_bt_waypoint_follower` (name subject to adjustment) that will allow you to create this application more easily. In this school of thought, the waypoint following application is more closely tied to the system autonomy, or in many cases, is the system autonomy.

Neither is better than the other, it highly depends on the tasks your robot(s) are completing, in what type of environment, and with what cloud resources available. Often this distinction is very clear for a given business case.

`nav2_waypoint_follower` also supports GPS waypoint following when global localization is provided by [robot_localization](https://github.com/cra-ros-pkg/robot_localization/)  using the `navsat_transform` node - but also may be provided by Fuse or any number of other sources.
There is an action server named `/follow_gps_waypoints` within `nav2_waypoint_follower` that can directly take in goals expressed in GPS coordinates, convert them to cartesian goals in the global frame, and execute them as cartesian waypoints.
