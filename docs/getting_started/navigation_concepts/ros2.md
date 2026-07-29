# ROS 2 { #ros-2 }

ROS 2 is the core middleware used for Nav2. If you are unfamiliar with this, please visit [the ROS 2 documentation](https://docs.ros.org/en/lyrical/) before continuing.

## Action Server

Just as in ROS, action servers are a common way to control long running tasks like navigation.
This stack makes more extensive use of actions, and in some cases, without an easy topic interface.
It is more important to understand action servers as a developer in ROS 2.
Some simple CLI examples can be found in the [ROS 2 documentation](https://docs.ros.org/en/lyrical/Tutorials/Understanding-ROS2-Actions.html).

Action servers are similar to a canonical service server.
A client will request some task to be completed, except, this task may take a long time.
An example would be moving the shovel up from a bulldozer or ask a robot to travel 10 meters to the right.

In this situation, action servers and clients allow us to call a long-running task in another process or thread and return a future to its result.
It is permissible at this point to block until the action is complete, however, you may want to occasionally check if the action is complete and continue to process work in the client thread.
Since it is long-running, action servers will also provide feedback to their clients.
This feedback can be anything and is defined in the ROS `.action` along with the request and result types.
In the bulldozer example, a request may be an angle, a feedback may be the angle remaining to be moved, and the result is a success or fail boolean with the end angle.
In the navigation example, a request may be a position, a feedback may be the time its been navigating for and the distance to the goal, and the result a boolean for success.

Feedback and results can be gathered synchronously by registering callbacks with the action client.
They may also be gathered by asynchronously requesting information from the shared future objects.
Both require spinning the client node to process callback groups.

Action servers are used in this stack to communicate with the highest level Behavior Tree (BT) navigator through a `NavigateToPose` action message.
They are also used for the BT navigator to communicate with the subsequent smaller action servers to compute plans, control efforts, and recoveries.
Each will have their own unique `.action` type in `nav2_msgs` for interacting with the servers.

## Lifecycle Nodes and Bond

Lifecycle (or Managed, more correctly) nodes are unique to ROS 2.
More information can be [found here](https://design.ros2.org/articles/node_lifecycle.html).
They are nodes that contain state machine transitions for bringup and teardown of ROS 2 servers.
This helps in deterministic behavior of ROS systems in startup and shutdown.
It also helps users structure their programs in reasonable ways for commercial uses and debugging.

When a node is started, it is in the unconfigured state, only processing the node's constructor which should **not** contain any ROS networking setup or parameter reading.
By the launch system, or the supplied lifecycle manager, the nodes need to be transitioned to inactive by configuring.
After, it is possible to activate the node by transitioning through the activating stage.

This state will allow the node to process information and be fully setup to run.
The configuration stage, triggering the `on_configure()` method, will setup all parameters, ROS networking interfaces, and for safety systems, all dynamically allocated memory.
The activation stage, triggering the `on_activate()` method, will activate the ROS networking interfaces and set any states in the program to start processing information.

To shutdown, we transition into deactivating, cleaning up, shutting down and end in the finalized state.
The networking interfaces are deactivated and stop processing, deallocate memory, exit cleanly, in those stages, respectively.

The lifecycle node framework is used extensively through out this project and all servers utilize it.
It is best convention for all ROS systems to use lifecycle nodes if it is possible.

Within Nav2, we use a wrapper of LifecycleNodes, `nav2_util LifecycleNode`.
This wrapper wraps much of the complexities of LifecycleNodes for typical applications.
It also includes a `bond` connection for the lifecycle manager to ensure that after a server transitions up, it also remains active.
If a server crashes, it lets the lifecycle manager know and transition down the system to prevent a critical failure. See [Eloquent to Foxy][eloquent-to-foxy] for details.
