.. _writing_new_nbt_plugin:

Writing a New Behavior Tree Plugin
**********************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

Overview
========

This tutorial shows how to create your own behavior tree (BT) plugin.
The BT plugins are used as nodes in the behavior tree XML processed by the BT Navigator for navigation logic.

Requirements
============

- ROS 2 (binary or build-from-source)
- Nav2 (Including dependencies)
- Gazebo
- Turtlebot3

Tutorial Steps
==============

1- Creating a new BT Plugin
---------------------------

We will create a simple BT plugin node to perform an action on another server.
For this example, we're going to analyze the simplest behavior tree action node in the ``nav2_behavior_tree`` package, the ``wait`` node.
Beyond this example of an action BT node, you can also create custom decorator, condition, and control nodes.
Each node type has a unique role in the behavior tree to perform actions like planning, control the flow of the BT, check the status of a condition, or modify the output of other BT nodes.

The code in this tutorial can be found in `nav2_behavior_tree <https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree>`_ package as the ``wait_action`` node.
This action node can be considered as a reference for writing other action node plugins.

Our example plugin inherits from the base class ``nav2_behavior_tree::BtActionNode``.
The base class is a wrapper on the BehaviorTree.CPP ``BT::ActionNodeBase`` that simplifies BT action nodes that utilize ROS 2 action clients.
An ``BTActionNode`` is both a BT action and uses ROS 2 action network interfaces for calling a remote server to do some work.

When working with other types of BT nodes (e.g. decorator, control, condition) use the corresponding BT node, ``BT::DecoratorNode``, ``BT::ControlNode``, or ``BT::ConditionNode``.
For BT action nodes that do *not* utilize ROS 2 action interfaces, use the ``BT::ActionNodeBase`` base class itself.

The ``BTActionNode`` class provides 5 virtual methods to use, in addition to the information provided in the constructor.
Let's learn more about the methods needed to write a BT action plugin.

+----------------------+----------------------------------------------------------------------------+-------------------------+
| **method**           | **Method description**                                                     | **Required?**           |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| Constructor          | Constructor to indicate the corresponding XML tag name to that matches     | Yes                     |
|                      | the plugin, the name of the action server to call using the plugin,        |                         |
|                      | and any BehaviorTree.CPP special configurations required.                  |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| providedPorts()      | A function to define the input and output ports a BT node may have.        | Yes                     |
|                      | These are analogous to parameters that are defined in the BT XML           |                         |
|                      | by hardcoded values or by the value of output ports of other nodes.        |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| on_tick()            | Method is called when this BT node is ticked by the behavior tree while    | No                      |
|                      | executing. This should be used to get dynamic updates like new blackboard  |                         |
|                      | values, input ports, or parameters. May also reset state for the action.   |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| on_wait_for_result() | Method is called when the behavior tree node is waiting for a result from  | No                      |
|                      | the ROS 2 action server it called. This could be used to check for updates |                         |
|                      | to preempt the current task, check for a timeout, or anything to compute   |                         |
|                      | while waiting for the action to complete.                                  |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| on_success()         | Method is called when the ROS 2 action server returns a successful result. | No                      |
|                      | Returns the value the BT node will report back to the tree.                |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| on_aborted()         | Method is called when the ROS 2 action server returns an aborted result.   | No                      |
|                      | Returns the value the BT node will report back to the tree.                |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| on_cancelled()       | MMethod is called when the ROS 2 action server returns a cancelled result. | No                      |
|                      | Returns the value the BT node will report back to the tree.                |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+

For this tutorial, we will only be using the ``on_tick()`` method.

In the constructor, we need to get any non-variable parameters that apply to the behavior tree node.
In this example, we need to get the value of the duration to sleep from the input port of the behavior tree XML.

.. code-block:: c++

  WaitAction::WaitAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf)
  {
    int duration;
    getInput("wait_duration", duration);
    if (duration <= 0) {
      RCLCPP_WARN(
        node_->get_logger(), "Wait duration is negative or zero "
        "(%i). Setting to positive.", duration);
      duration *= -1;
    }

    goal_.time.sec = duration;
  }

Here, we give the input of the ``xml_tag_name``, which tells the BT node plugin the string in the XML that corresponds to this node.
This will be seen later when we register this BT node as a plugin.
It also takes in the string name of the action server that it will call to execute some behavior.
Finally, a set of configurations that we can safely ignore for the purposes of most node plugins.

We then call the ``BTActionNode`` constructor. As can be seen, it's templated by the ROS 2 action type, so we give it the ``nav2_msgs::action::Wait`` action message type and forward our other inputs.
The ``BTActionNode`` has the ``tick()`` method, which is called directly by the behavior tree when this node is called from the tree.
``on_tick()`` is then called before sending the action client goal.

In the body of the constructor, we get the input port ``getInput`` of the parameter ``wait_duration``, which can be configured independently for every instance of the ``wait`` node in the tree.
It is set in the ``duration`` parameter and inserted into the ``goal_``.
The ``goal_`` class variable is the goal that the ROS 2 action client will send to the action server.
So in this example, we set the duration to the time we want to wait by so that the action server knows the specifics of our request.

The ``providedPorts()`` method gives us the opportunity to define input or output ports.
Ports can be thought of as parameters that the behavior tree node has access to from the behavior tree itself.
For our example, there is only a single input port, the ``wait_duration`` which can be set in the BT XML for each instance of the ``wait`` recovery.
We set the type, ``int``, the default ``1``, the name ``wait_duration``, and a description of the port ``Wait time``.

.. code-block:: c++

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<int>("wait_duration", 1, "Wait time")
      });
  }


The ``on_tick()`` method is called when the behavior tree ticks a specific node.
For the wait BT node, we simply want to notify a counter on the blackboard that an action plugin that corresponds to a recovery was ticked.
This is useful to keep metrics about the number of recoveries executed during a specific navigation run.
You could also log or update the ``goal_`` waiting duration if that is a variable input.

.. code-block:: c++

  void WaitAction::on_tick()
  {
    increment_recovery_count();
  }

The remaining methods are not used and are not mandatory to override them.
Only some BT node plugins will require overriding ``on_wait_for_result()`` to check for preemption or check a timeout.
The success, aborted, and cancelled methods will default to ``SUCCESS``, ``FAILURE``, ``SUCCESS`` respectively, if not overridden.

2- Exporting the planner plugin
-------------------------------

Now that we have created our custom BT node, we need to export our plugin so that it would be visible to the behavior tree when it loads a custom BT XML.
Plugins are loaded at runtime, and if they are not visible, then our BT Navigator server won't be able to load them or use them.
In BehaviorTree.CPP, exporting and loading plugins is handled by the ``BT_REGISTER_NODES`` macro.

.. code-block:: c++
  
  BT_REGISTER_NODES(factory)
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::WaitAction>(name, "wait", config);
      };

    factory.registerBuilder<nav2_behavior_tree::WaitAction>("Wait", builder);
  }

In this macro, we must create a ``NodeBuilder`` so that our custom action node can have a non-default constructor signature (for the action and xml names).
This lambda will return a unique pointer to the behavior tree node we have created.
Fill in the constructor with the relevant information, giving it the ``name`` and ``config`` given in the function arguments.
Then define the ROS 2 action server's name that this BT node will call, in this case, it's the ``Wait`` action.

We finally give the builder to a factory to register.
``Wait`` given to the factory is the name in the behavior tree XML file that corresponds to this BT node plugin.
An example can be seen below, where the ``Wait`` BT XML node specifies a non-variable input port ``wait_duration`` of 5 seconds.

.. code-block:: xml

  <Wait wait_duration="5"/>

3- Add plugin library name to config
------------------------------------

In order for the BT Navigator node to discover the plugin we've just registered, we need to list the plugin library name under the bt_navigator node in the configuration YAML file. Configuration should look similar to the one shown below. Take note of nav2_wait_action_bt_node listed under plugin_lib_names.

.. code-block:: text

  bt_navigator:
    ros__parameters:
      use_sim_time: True
      global_frame: map
      robot_base_frame: base_link
      odom_topic: /odom
      default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
      plugin_lib_names:
      - nav2_back_up_action_bt_node # other plugin 
      - nav2_wait_action_bt_node    # our new plugin

4- Run Your Custom plugin
-------------------------

Now you can use a behavior tree with your custom BT node.
For example, the ``navigate_w_replanning_and_recovery.xml`` file is shown below.

Select this BT XML file in your specific navigation request in ``NavigateToPose`` or as the default behavior tree in the BT Navigator's configuration yaml file.

.. code-block:: xml

  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <RecoveryNode number_of_retries="6" name="NavigateRecovery">
        <PipelineSequence name="NavigateWithReplanning">
          <RateController hz="1.0">
            <RecoveryNode number_of_retries="1" name="ComputePathToPose">
              <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </RecoveryNode>
          </RateController>
          <RecoveryNode number_of_retries="1" name="FollowPath">
            <FollowPath path="{path}" controller_id="FollowPath"/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
          </RecoveryNode>
        </PipelineSequence>
        <ReactiveFallback name="RecoveryFallback">
          <GoalUpdated/>
          <SequenceStar name="RecoveryActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
            <Spin spin_dist="1.57"/>
            <Wait wait_duration="5"/>
          </SequenceStar>
        </ReactiveFallback>
      </RecoveryNode>
    </BehaviorTree>
  </root>
