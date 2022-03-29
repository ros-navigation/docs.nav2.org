.. _lifecycle_composition:

Setting Up Lifecycle and Composition Nodes
##########################################

In this guide, we will discuss two new concepts in ROS 2, namely Lifecycle and Composition. In the first section of this guide, we discuss the concept of Lifecyle in Nav2 and how to add a lifecycle nodes to make use of it. In the second section, we discuss the purpose of Composition and how to implement it manually and dynamically in Nav2.


Lifecycle
**********

Lifecycle is introduced in ROS 2 to systematically manage the bringup and shutdown of the different nodes involved in the robot's operation. The use of Lifecycle nodes ensures that all nodes are successfully instantiated before they begin their execution and Nav2 shuts down all nodes if there is any unresponsive node. 


Lifecycle nodes contain state machine transitions that enable deterministic behavior in ROS 2 servers. The Lifecycle node transitions in Nav2 are handled by the ``Lifecycle Manager``. The Lifecycle Manager transitions the states of the Lifecycle nodes and provides greater control over the state of a system.


The primary states of a Lifecycle node are ``Unconfigured``, ``Inactive``, ``Active``, and ``Finalized``. A Lifecycle node starts in an ``Unconfigured`` state after being instantiated. The Lifecycle Manager transitions a node from ``Unconfigured`` to ``Inactive`` by implementing the ``Configurating`` transition. The ``Configurating`` transition sets up all configuration parameters and prepares any required setup such as memory allocation and the set up of the static publication and subscription topics. A node in the ``Inactive`` state is allowed to reconfigure its parameters and but cannot perform any processing. From the ``Inactive`` state, the Lifecyle Manager implements the ``Activating`` transition state to transition the node from ``Inactive`` to ``Active``, which is the main state. A node in the ``Active`` state is allowed to perform any processing operation. In case a node crashes, the Lifecycle Manager shuts down the system to prevent any critical failures. On shutdown, the necessary cleanup operations are performed and the nodes are transitioned to the ``Finalized`` state via ``Deactivating``, ``CleaningUp``, and ``ShuttingDown`` transition states.

.. seealso::
    For more information on Lifecycle management, see the article on `Managed Nodes <https://design.ros2.org/articles/node_lifecycle.html>`_.

The Lifecycle node framework is heavily used by the Nav2 servers, such as the planner and controller servers which were discussed in the previous tutorial. 

You may wish to integrate your own nodes into the Nav2 framework or add new lifecycle nodes to your system. As an example, we will add a new notional lifecycle node ``sensor_driver``, and have it be controlled via the Nav2 Lifecycle Manager to ensure sensor feeds are available before activating navigation. You can do so by adding a ``sensor_driver`` node in your launch file and adding it to the list of nodes to be activated by the ``lifecycle_manager`` before navigation, as shown in the example below. 

.. code-block:: python
    
    lifecycle_nodes = ['sensor_driver',
                       'controller_server',
                       'smoother_server',
                       'planner_server',
                       'recoveries_server',
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

Composition is another new concept in ROS 2 that was introduced to reduce the memory and CPU resources by putting multiple nodes in a single process. In Nav2, Composition can be used to compose all Nav2 nodes in a single process instead of launching them separately. This is useful for deployment on embedded systems where developers need to optimize resource usage.

.. seealso::
   More information on Composition can be found `here <https://docs.ros.org/en/galactic/Tutorials/Composition.html>`_.

In the following subsections, we give an example on how to add a new Nav2 server, which we notionally call the ``route_server``, to our system. This can be done either through manual composition or dynamic composition.

Manual Composition
==================
For manual composition, there is an existing manually composed bringup file, `composed_bringup.cpp <https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/src/composed_bringup.cpp>`_, in ``nav2_bringup``. You can add ``route_server`` to this existing manual composition by doing the following steps:


1. Add the following to `composed_bringup.cpp <https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/src/composed_bringup.cpp>`_:
    
    Add the the header file of ``route_server``. 
    
    .. code-block:: cpp  

        #include "nav2_route_server/route_server.hpp"

    Create shared pointer ``route_node`` and add it to ``navigation_node_names`` to transition up with the lifecycle manager, if a lifecycle node. 


    .. code-block:: cpp  

         auto route_node = std::make_shared<nav2_route_server::RouteServer>();
         navigation_node_names.push_back(route_node->get_name());

    
    Add ``route_node`` to ``threads`` to create single thread executor for ``route_node``.

    .. code-block:: cpp  

         threads.push_back(create_spin_thread(route_node));
   

2. Add the package containing the server as a dependency to your ``package.xml`` file.

    .. code-block:: xml  

        <exec_depend>nav2_route_server</exec_depend>

3. Include the package in the ``CMakeLists.txt`` file.

    .. code-block:: xml
        
        find_package(nav2_route_server REQUIRED)
        set(dependencies nav2_route_server)

Dynamic Composition
===================
In dynamic composition, we make use of the launch files to compose different servers into a single process. The process is established by the ``ComposableNodeContainer`` container that is populated with composition nodes via ``ComposableNode``. This container can then be launched and used the same as any other Nav2 node.

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

Conclusion
**********

In this section of the guide, we have discussed Lifecycle and Composition nodes which are new and important concepts in ROS 2. We also showed how to implement Lifecycle and Composition to your newly created nodes/servers with Nav2. These two concepts are helpful to efficiently run your system and therefore are encouraged to be used throughout Nav2. 