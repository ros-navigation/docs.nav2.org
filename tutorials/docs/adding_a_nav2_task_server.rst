.. _adding_a_nav2_task_server:

Adding a New Nav2 Task Server 
#############################

A nav2 task server consists of server side logic to complete different types of requests, usually called by the autonomy system or through the Behavior Tree Navigator. In this guide, we will discuss the core components needed to add a new task server to Nav2 (ex. Controller, Behavior, Smoother, Planner Servers). Namely, how to set up your new Lifecycle-Component Node for launch and state management and the communication of semantically meaningful error codes (if necessary). 

While this tutorial does not cover how to add the complementary Behavior Tree Node to interact with this new Task Server, that is covered at length in :ref:`writing_new_nbt_plugin` so this Task Server can be invoked in the BTs in BT Navigator.

If you've created a new Task Server that may have general reuse for the community, consider contacting the maintainers to add it to the Nav2 project! Nav2 gets better by contributions by users like you!



Lifecycle Nodes
***************

The Lifecycle node is the first key component of a nav2 task server. Lifecycle nodes were introduced in ROS 2 to systematically manage the bringup and shutdown of the different nodes involved in the robot's operation. The use of Lifecycle nodes ensures that all nodes are successfully instantiated before they begin their execution and Nav2 shuts down all nodes if there is any unresponsive node.


Lifecycle nodes contain state machine transitions that enable deterministic behavior in ROS 2 servers. The Lifecycle node transitions in Nav2 are handled by the ``Lifecycle Manager``. The Lifecycle Manager transitions the states of the Lifecycle nodes and provides greater control over the state of a system.


The primary states of a Lifecycle node are ``Unconfigured``, ``Inactive``, ``Active``, and ``Finalized``. A Lifecycle node starts in an ``Unconfigured`` state after being instantiated. The Lifecycle Manager transitions a node from ``Unconfigured`` to ``Inactive`` by implementing the ``Configurating`` transition. The ``Configurating`` transition sets up all configuration parameters and prepares any required setup such as memory allocation and the set up of the static publication and subscription topics. A node in the ``Inactive`` state is allowed to reconfigure its parameters and but cannot perform any processing. From the ``Inactive`` state, the Lifecyle Manager implements the ``Activating`` transition state to transition the node from ``Inactive`` to ``Active``, which is the main state. A node in the ``Active`` state is allowed to perform any processing operation. In case a node crashes, the Lifecycle Manager shuts down the system to prevent any critical failures. On shutdown, the necessary cleanup operations are performed and the nodes are transitioned to the ``Finalized`` state via ``Deactivating``, ``CleaningUp``, and ``ShuttingDown`` transition states.

.. seealso::
    For more information on Lifecycle management, see the article on `Managed Nodes <https://design.ros2.org/articles/node_lifecycle.html>`_.

You may wish to integrate your own nodes into the Nav2 framework or add new lifecycle nodes to your system. As an example, we will add a new notional lifecycle node ``sensor_driver``, and have it be controlled via the Nav2 Lifecycle Manager to ensure sensor feeds are available before activating navigation. You can do so by adding a ``sensor_driver`` node in your launch file and adding it to the list of nodes to be activated by the ``lifecycle_manager`` before navigation, as shown in the example below.

.. code-block:: python

    lifecycle_nodes = ['sensor_driver',
                       'controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    ...

    Node(
        package='nav2_sensor_driver',
        executable='sensor_driver',
        name='sensor_driver',
        output='screen',
        parameters=[configured_params],
        remappings=remappings),

    Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': autostart},
                    {'node_names': lifecycle_nodes}]),


In the snippet above, the nodes to be handled by the Lifecycle Manager are set using the ``node_names`` parameter. The ``node_names`` parameter takes in an ordered list of nodes to bringup through the Lifecycle transition. As shown in the snippet, the ``node_names`` parameter takes in ``lifecycle_nodes`` which contains the list of nodes to be added to the Lifecycle Manager. The Lifecycle Manager implements bringup transitions (``Configuring`` and ``Activating``) to the nodes one-by-one and in order, while the nodes are processed in reverse order for shutdown transitions. Hence, the ``sensor_driver`` is listed first before the other navigation servers so that the sensor data is available before the navigation servers are activated.



Two other parameters of the Lifecycle Manager are ``autostart`` and ``bond_timeout``. Set ``autostart`` to ``true`` if you want to set the transition nodes to the ``Active`` state on startup. Otherwise, you will need to manually trigger Lifecycle Manager to transition up the system. The ``bond_timeout`` sets the waiting time to decide when to transition down all of the nodes if a node is not responding.

.. note::
   More information on Lifecycle Manager parameters can be found in the `Configuration Guide of Lifecycle Manager <https://navigation.ros.org/configuration/packages/configuring-lifecycle.html>`_


Composition
***********

Composition is the second key component nav2 task servers that was introduced to reduce the memory and CPU resources by putting multiple nodes in a single process. In Nav2, Composition can be used to compose all Nav2 nodes in a single process instead of launching them separately. This is useful for deployment on embedded systems where developers need to optimize resource usage.

.. seealso::
   More information on Composition can be found `here <https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html>`_.

In the following section, we give an example on how to add a new Nav2 server, which we notionally call the ``route_server``, to our system.


We make use of the launch files to compose different servers into a single process. The process is established by the ``ComposableNodeContainer`` container that is populated with composition nodes via ``ComposableNode``. This container can then be launched and used the same as any other Nav2 node.

1. Add a new ``ComposableNode()`` instance in your launch file pointing to the component container of your choice.

    .. code-block:: python

        container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_route_server',
                    plugin='nav2_route_server::RouteServer',
                    name='nav2_route_server'),
            ],
            output='screen',
        )

    .. seealso::
        See example in composition demo's `composition_demo.launch.py <https://github.com/ros2/demos/blob/master/composition/launch/composition_demo.launch.py>`_.

2. Add the package containing the server to your ``package.xml`` file.

    .. code-block:: xml

        <exec_depend>nav2_route_server</exec_depend>


Error codes 
***********

Your nav2 task server may also wish to return a 'error_code' in its action response (though not required). If there are semantically meaningful and actionable types of failures for your system, this is a systemic way to communicate those failures which may be automatically aggregated into the responses of the navigation system to your application.

It is important to note that error codes from 0-9999 are reserved for internal nav2 servers with each server offset by 100 while external servers start at 10000 and end at 65535. 
The table below shows the current servers along with the expected error code structure.



+---------------------------------------------------+-----------------------+----------------------+
| Server Name                                       | Reserved              | RANGE                |
+===================================================+=======================+======================+
| ...                                               | NONE=0, UNKNOWN=1     | 2-99                 |
+---------------------------------------------------+-----------------------+----------------------+
| `Controller Server`_                              | NONE=0, UNKNOWN=100   | 101-199              |
+---------------------------------------------------+-----------------------+----------------------+
| `Planner Server`_ (compute_path_to_pose)          | NONE=0, UNKNOWN=200   | 201-299              |
+---------------------------------------------------+-----------------------+----------------------+
| `Planner Server`_ (compute_path_through_poses)    | NONE=0, UNKNOWN=300   | 301-399              |
+---------------------------------------------------+-----------------------+----------------------+
| ...                                               | ...                   |                      |
+---------------------------------------------------+-----------------------+----------------------+
| `Smoother Server`_                                | NONE=0, UNKNOWN=500   | 501-599              |
+---------------------------------------------------+-----------------------+----------------------+
| `Waypoint Follower Server`_                       | NONE=0, UNKNOWN=600   | 601-699              |
+---------------------------------------------------+-----------------------+----------------------+
| `Behavior Server`_                                | NONE=0                | 701-799              |
+---------------------------------------------------+-----------------------+----------------------+
| ...                                               | ...                   |                      |
+---------------------------------------------------+-----------------------+----------------------+
| Last Nav2 Server                                  | NONE=0, UNKNOWN=9900  | 9901-9999            |
+---------------------------------------------------+-----------------------+----------------------+
| First External Server                             | NONE=0, UNKNOWN=10000 | 10001-10099          |
+---------------------------------------------------+-----------------------+----------------------+
| ...                                               | ...                   |                      |
+---------------------------------------------------+-----------------------+----------------------+

.. _Controller Server: https://github.com/ros-planning/navigation2/blob/main/nav2_controller/src/controller_server.cpp
.. _Planner Server: https://github.com/ros-planning/navigation2/blob/main/nav2_planner/src/planner_server.cpp
.. _Smoother Server: https://github.com/ros-planning/navigation2/blob/main/nav2_smoother/src/nav2_smoother.cpp
.. _Waypoint Follower Server: https://github.com/ros-planning/navigation2/blob/main/nav2_waypoint_follower/src/waypoint_follower.cpp
.. _Behavior Server: https://github.com/ros-planning/navigation2/blob/main/nav2_behaviors/src/behavior_server.cpp

Error codes are attached to the response of the action message. An example can be seen below for the route server. Note that by convention we set the error code field within the message definition to ``error_code``.



.. code-block:: bash

    # Error codes
    # Note: The expected priority order of the errors should match the message order
    uint16 NONE=0 # 0 is reserved for NONE
    uint16 UNKNOWN=10000 # first error code in the sequence is reserved for UNKNOWN

    # User Error codes below
    int16 INVILAD_START=10001
    int16 NO_VALID_ROUTE=10002

    #goal definition
    route_msgs/PoseStamped goal
    route_msgs/PoseStamped start
    string route_id
    ---
    #result definition
    nav_msgs/Route route
    builtin_interfaces/Duration route_time
    uint16 error_code
    ---

As stated in the message, the priority order of the errors should match the message order, 0 is reserved for NONE and the first error code in the sequence is reserved for UNKNOWN.
Since the the route server is a external server, the errors codes start at 10000 and go up to 10099.

In order to propigate your server's error code to the rest of the system it must be added to the nav2_params.yaml file. 
The `error_code_id_names` inside of the BT Navigator define what error codes to look for on the blackboard by the server. The lowest error code of the sequence is then returned - whereas the code enums increase the higher up in the software stack - giving higher priority to lower-level failures.



.. code-block:: yaml

    error_code_id_names:
        - compute_path_error_code_id
        - follow_path_error_code_id
        - route_error_code_id

Conclusion
**********

In this section of the guide, we have discussed Lifecycle Nodes, Composition and Error Codes which are new and important concepts in ROS 2. We also showed how to implement Lifecycle Nodes, Composition and Error Codes to your newly created nodes/servers with Nav2. These three concepts are helpful to efficiently run your system and therefore are encouraged to be used throughout Nav2.
