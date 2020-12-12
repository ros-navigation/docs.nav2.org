.. _groot_introduction:

Groot - Interacting with your BehaviorTree
******************************************
- `Overview`_ 
- `Edit Custom Behavior Trees`_
- `Creating your Custom Node`_
- `Live Monitoring Behavior Trees`_

Overview
========

.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/Z6xCat0zaWU?autoplay=1&mute=1" frameborder="0" allowfullscreen></iframe>
        </div>
      </div>
    </h1>



`Groot <https://github.com/BehaviorTree/Groot>`_ is the companion application of the C++ library
`BehaviorTree.CPP <https://github.com/BehaviorTree/BehaviorTree.CPP>`_ to create, edit, and monitor behavior trees.
Behavior Trees are deeply integrated into Nav2, while not being strictly ROS dependent. Behavior Trees, in short BTs, consist out of multiples leave nodes.
Such leave nodes must be type of either `Action`, `Condition`, `Control`, `Decorator`, or `SubTree`. 
These types are described in more detail at the chapter `Creating your Custom Node`_.

Each node must be written in C++ and can be loaded as plugins into a factory. 
The `BehaviorTree Documentation <../../plugin_tutorials/docs/writing_new_bt_plugin.html>`_ offers well written examples of creating a simple node, that make use of existing Macros for convenient adding them into your Behavior Tree.
Dynamic at runtime and C++ are perfectly combined through the underlying builder-factory mechanism and class-loader mechanics of plugins provided through shared-objects.

While it is crucial to correctly implement the individual nodes, the composition of the individual nodes is as least as important. 
BehaviorTree.Cpp brings a lot of dynamic with it. This dynamic can be controlled with XML-files that hold the construction plan for the integrated factory.
XML in this case only describes the logic how the different nodes are combined through an own Domain specific langugage.
Groot basically helps to visually understand and work with those special XML-files. The XML Format is defined `in detail here <https://www.behaviortree.dev/xml_format/>`_.

Groot can not only visualize the XML describing the BT, but also edit and even create new behavior trees. Additionally, complete new custom nodes can be described using Groot.
This helps to describe the functionality of your application at a higher abstraction layer. 
If ROS is your given Middleware in this case, BTs can be described as "Top-Ware" and Groot helps you visualize it similar to a down stripped RVIz.
`Groot support for Nav2 <https://github.com/ros-planning/navigation2/pull/1958>`_ is integrated into Nav2 with the latest version of BT.cpp V3.

In the video above you can see Groot side-by-side with RVIz and a test platform 100% equipped with ROS-enabled hardware from SIEMENS.
Groot not only displays the current BehaviorTree, but also shows the current active nodes in near real-time, all while the real robot operates.
Understanding the visual output of Groot will be explained in chapter `Live Monitoring Behavior Trees`_.


Edit Custom Behavior Trees
==========================

+-----------------------------------------------------------+
| .. figure:: images/Groot/bt_w_replanning_and_recovery.png |
|   :name: groot_nav2_default_bt                            |
|                                                           |
|   Full Nav2 Default BehaviorTree                          |
+-----------------------------------------------------------+

In order to display a BehaviorTree like in :numref:`groot_nav2_default_bt`, we first have to start Groot in the Editor mode.
After this, Groot can already display BehaviorTrees consisting out of default nodes from the BT.CPP V3 library.
Therefore, a few more steps are necessary to view and edit existing BehaviorTrees from Nav2 with existing Custom Nodes:


1. Open Groot in editor mode, now Groot should look like in :numref:`groot_bt_editor`.
2. Select the `Load palette from file` option either via context menu or the import icon in the top middle of the menu bar.

+-----------------------------------------------------------+-----------------------------------------------------------+
| .. figure:: images/Groot/groot_bt_editor.png              | .. figure:: images/Groot/groot_with_nav2_custom_nodes.png |
|   :name: groot_bt_editor                                  |   :name: groot_bt_editor_with_nodes                       |
|                                                           |                                                           |
|   Default Editor View                                     |   Editor with Custom Nodes loaded in blue                 |
+-----------------------------------------------------------+-----------------------------------------------------------+


3. Open the file `/path/to/navigation2/nav2_behavior_tree/nav2_tree_nodes.xml` to import all the custom behavior tree nodes used for navigation. Now Groot should look like in :numref:`groot_bt_editor_with_nodes`.
4. Select `Load tree` option near the top left corner
5. Browse the tree you want to visualize, then select ok. The Nav2 BTs exist in `/path/to/navigation2/nav2_bt_navigator/behavior_trees/`

In case you choose the default tree `navigate_w_replanning_and_recovery.xml`, then your Groot editor should display :numref:`groot_nav2_default_bt`.

.. note::
  If a tree cannot be visualized because some nodes are missing in the pallet, you might need to visit the BehaviorTree section of :ref:`plugins` and search for the plugin that is missing.
  If you got all the optional blackboard fields from the code of the plugin, you are free to go to add the description to ``/path/to/navigation2/nav2_behavior_tree/nav2_tree_nodes.xml``.
  You can create the missing xml description of that particular node by following the steps of `Creating your Custom Node`_.
  Now you can reload the pallet from this updated file and should be able to work with the tree.

Creating your Custom node
=========================

+-----------------------------------------------------------+
| .. figure:: images/Groot/groot_create_custom_node.png     |
|   :name: groot_create_custom_node                         |
|                                                           |
|   Create a new Custom Node                                |
+-----------------------------------------------------------+

Remember, there exists 5 different categories for Nodes, they are called `Action`, `Condition`, `Control`, `Decorator`, and `SubTree`.
Each of them holds a special functionality. Detailed descriptions can be taken from the `official BehaviorTree Documentation <https://www.behaviortree.dev/bt_basics/#types-of-nodes>`_.

The Groot specification of any of those nodes help in an early design process for your BT. 
It also guides the user on setting any Input or Output-Ports conveniently.
Both, the node-palette and the resulting BehaviorTree (composition of nodes) can profit from using Groot.
But, only creating a new node in Groot is not all that you need, except for SubTrees that can be fully designed with Groot.
Try seeing Groot as a tool that complies to all the `BT specific xml-format rules <https://www.behaviortree.dev/xml_format/>`_.

Implementing the C++ features of your node needs to be done separately from Groot, which is described in :ref:`writing_new_nbt_plugin`.

Creating a new custom node can be started by clicking the orange marked icon in :numref:`groot_create_custom_node`, while Groot is in Editor mode.
This should load a new window, similar to :numref:`groot_interactive_node_creation`. 
In this new window multiple small steps must be fulfilled, in order to create a new custom node. First, a name must be defined in the green field. 
Second, the type of the new node must be defined in the orange drop down menu. 
Optionally, input and output-ports can be defined for data exchange on the blackboard. 
They are managed trough the buttons in the blue box.
After pressing `OK` in :numref:`groot_interactive_node_creation`, the new custom node should appear in blue in the `TreeNode Palette` as in :numref:`groot_export_new_node`.

+--------------------------------------------------------------+--------------------------------------------------------------+
| .. figure:: images/Groot/groot_interactive_node_creation.png | .. figure:: images/Groot/groot_export_new_node.png           |
|   :name: groot_interactive_node_creation                     |   :name: groot_export_new_node                               |
|                                                              |   :width: 180%                                               |
|                                                              |                                                              |
|   UI to describing new Nodes                                 |   Exporting the new Custom Node                              |
+--------------------------------------------------------------+--------------------------------------------------------------+

Before starting to create a new BT based on the new custom nodes, it is recommend to export the newly created nodes into the special XML-format.
This can be performed with the icon highlighted in green from :numref:`groot_export_new_node`.

It is important to safe new custom TreeNodes if it is intended to reuse them or display BT based on those custom nodes in the Editor mode.
The resulting XML output from the node created in :numref:`groot_interactive_node_creation` can be seen below.


.. code-block:: xml

  <root>
      <TreeNodesModel>
          <Action ID="MyAwesomeNewNode">
              <input_port name="key_name" default="false">coffee</input_port>
              <output_port name="key_name2" default="42">Sense of life</output_port>
              <inout_port name="next_target" default="pancakes">rolling target</inout_port>
          </Action>
      </TreeNodesModel>
  </root>


Live Monitoring Behavior Trees
==============================

+-------------------------------------------------------------------------------------------------------------------------------------------------+
| .. raw:: html                                                                                                                                   |
|                                                                                                                                                 |
|   <a href="https://gifyu.com/image/cFdR"><img src="https://s7.gifyu.com/images/Groot-Test-5FPS.gif" alt="Groot-Test-5FPS.gif" border="0" /></a> |
|                                                                                                                                                 |
+-------------------------------------------------------------------------------------------------------------------------------------------------+

In the world of complex robotics, an engineer needs a broad variety of tools to be prepared for unforeseen failures of an expected system behavior.
While RVIz and other CLI-based tools might give you great insight into the ROS-Middleware, including sensor data and dedicated service/action-calls,
a tool providing insights into a more abstract condition of a robot was needed.
In combination with BehaviorTrees, that offer new possibilities in describing methods to solve tasks and how to handle errors, 
Groot tries to fill the gap for this tool. You can see RVIz and Groot together in action in the gif above or in the initial video sequence.
For demonstration purposes, a few system-errors - wrong pose set - were introduced with RVIz to highlight changing BehaviorTree states.

The BehaviorTree above shows the current default BehaviorTree of Nav2. 
In short, the left side is responsible for the default navigation towards a goal and the right side tries to handle errors during the navigation process.
With the help of live monitoring, it can not only be seen which node(s) are currently active - orange color -, but also which nodes returned a `SUCCESS` and which a `FAILURE` response code.
For instance here the different stages of the recovery behavior can be observed, while one after one node turns green.

As described, with Groot one can see in addition to tools like RVIz, at which certain condition a given task failed.
This enables a complete new way of debugging, as the operator or engineer does not need to know the complete robot architecture off his head.
BehaviorTrees cannot give a deep view inside the code-stack, but they can help pinpointing problems due to the modular architecture enforced by BTs.

In order to enable live monitoring for your robot, a few easy steps are necessary.

Behavior trees of the BT.cpp V3 library do not automatically include monitoring. 
To enable monitoring with Groot, an additional logger has to be attached to the BT.
For communication between the BT and Groot, the middleware `ZMQ` has been chosen.

Monitoring with Groot in Nav2 is enabled by default. 
It can be deactivated by setting ``enable_groot_monitoring`` to `false`, as seen in :ref:`configuring_bt_navigator`.
If desired, the `ZMQ` network ports for ``zmq-server`` and ``zmq-publisher`` can be set optionally. 
The defaults correspond to the default networking-ports within Groot.

To try the live monitoring features, a few prerequisites have to be met.
Given the launch-parameters are set correctly, the navigation2 stack has to be started first.
Regardless of simulation or real hardware, it is important to note that **monitoring only works if the behavior tree is currently running!**

A step-by-step guide for simulation can look like this:

  1. Complete :ref:`getting_started` and be able to run the tb3 simulation
  2. Check if ``enable_groot_monitoring`` is set to ``True`` in the ``params.yaml`` file in ``nav2_bringup/bringup/params``
  3. Start the tb3 simulation
  4. Set the initial pose of the robot -> this will activate the whole navigation2 stack
  5. Start Groot and choose the monitor mode
  6. Press connect in the upper left corner (``Server IP``, ``Publisher Port``, and ``Server Port`` can all be left to the default for the simulation)
  7. The behavior tree should be visible now in Groot
  8. Send a new goal to your robot through RVIz
  9. Watch your robot drive in simulation and see how Groot automatically monitors the state of your behavior tree 


Real world robots can easily be adapted to this. Just change the ``Server IP`` 
and optionally different zmq network ports accordingly to your local environment.

Reloading of the behavior tree in Groot is done on multiple occasions and works completely automatically. 
Sending a new goal with a different BT.xml will also trigger Groot to reload the new BT to display and monitor.
More about `Groot reloading the BT <https://github.com/BehaviorTree/Groot/pull/96>`_ can be seen in the merged PR here.


.. note::
  Monitoring mode is unaffected from failing to display BTs with custom nodes that are not already imported into Groot, unlike the Editor Mode.
  This is due the fact that the whole BT-factory with sufficient meta data gets transferred upon the first connection with Groot.
  There is a known issue to reduce the overhead of creating additional xml-node-description files, when the data is also available in the shared object.
  Feel free to support adding this feature to the open source project Groot. The `feature request <https://github.com/BehaviorTree/Groot/issues/10>`_ already exists for this.
