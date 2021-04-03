.. _custom_behavior_tree:

Creating a Custom Behavior Tree
*******************************

- `Overview`_
- `Prerequisites`_
- `Introduction To Nav2 Specific Nodes`_
- `Navigate With Replanning and Recovery`_
- `Navigation Subtree`_
- `Recovery Subtree`_
- `Custom Action`_
- `Adding to Launch File`_
- `Testing`_

Overview
========

This document serves as a reference guide to the main behavior tree (BT) used in Nav2,
and explains the process for customizing this BT.

There are many example behavior trees provided in ``nav2_bt_navigator/behavior_trees``,
but these sometimes have to be re-configured based on the application of the robot. 
The following tutorial will walk through the current main default BT ``navigate_w_replanning_and_recovery.xml``
and will show users how to modify this BT in potentially useful ways, using the example of developing a BT that follows a predefined path.

Prerequisites
=============

- Have a valid installation of Nav2 (see the `getting started guide <../../getting_started/index.html>`_)

- Have a robot (simulated, or physical) that can be used for testing that can already navigate with Nav2

- Become familiar with the concept of a behavior tree before continuing with this tutorial
  
    - Read the short explanation in `navigation concepts <../../concepts/index.html>`_
  
    - Read the general tutorial ang guide (not Nav2 specific) on the `BehaviorTree CPP V3 <https://www.behaviortree.dev/>`_ website. Specifically, the "Learn the Basics" section on the BehaviorTree CPP V3 website explains the basic generic nodes that will be used that this guide will build upon.

Introduction To Nav2 Specific Nodes
===================================
.. warning::
    Vocabulary can be a large point of confusion here when first starting out.
        - A ``Node`` when discussing BT is entirely different than a ``Node`` in the ROS2 context
  
        - A ``Recovery`` in the context of BT is different than a navigation ``Recovery`` behavior
  
        - An ``ActionNode`` in the context of BT is not necessarily connected to an Action Server in the ROS2 context (but often it is)

There are quite a few custom Nav2 BT nodes that are provided to be used in the Nav2 specific fashion. Some commonly used Nav2 nodes will be described below.
The full list of custom BT nodes can be found in the `nav2_behavior_tree plugins folder <https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins>`_.
The `configuration guide <../../configuration/packages/bt-plugins/configuring-bt-xml.html>`_ can also be quite useful.

Action Nodes
------------

- ComputePathToPose - ComputePathToPose Action Server Client (Planner Interface)

- FollowPath - FollowPath Action Server Client (Controller Interface)

- Spin, Wait, Backup - Recoveries Action Server Client

- ClearCostmapService - ClearCostmapService Server Clients

Upon completion, these action nodes will return ``SUCCESS`` if the action server believes the action has been completed correctly, ``RUNNING`` when still running, and will return ``FAILURE`` otherwise. Note that in the above list,
the `ClearCostmapService` action node is *not* an action server client, but a service client.

Condition Nodes
---------------

- GoalUpdated

- GoalReached

- InitialPoseReceived

- isBatteryLow

The above list of condition nodes can be used to probe particular aspects of the system. Typically they will return ``SUCCESS`` is ``TRUE`` and ``FAILURE`` when ``FALSE``.
The key condition that is used in the default Nav2 BT is ``GoalUpdated`` which is checked asynchronously within particular subtrees. This condition node allows for the behavior described as "If the goal has updated, then we must replan". 
Condition nodes are typically paired with ReactiveFallback nodes.

Decorator: Rate Controller
--------------------------
The rate controller node helps control the ticking of its children nodes. The tick rate is an exposed blackboard parameter, it is being used in the default Nav2 BT to limit the rate at which the ``ComputePathToPose`` action node is called.

Control: PipelineSequence
-------------------------
The PipelineSequence control node re-ticks previous children when a child returns ``RUNNING``.
This node is similar to the ``Sequence`` node, with the additional property that the children prior to the "current" are re-ticked, (resembling the flow of water in a pipe).
If at any point a child returns ``FAILURE``, all children will be halted and the parent node will also return ``FAILURE``. Upon ``SUCCESS`` of the last node in the sequence, this node will halt and return ``SUCCESS``.

To explain this further, here is an example BT that uses PipelineSequence.

|

 .. image:: images/custom_behavior_tree/control_pipelineSequence.png
    :align: center

|                  

.. code-block:: xml

    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <PipelineSequence>
                <Action_A/>
                <Action_B/>
                <Action_C/>
            </PipelineSequence>
        </BehaviorTree>
    </root>

1. ``Action_A``, ``Action_B``, and ``Action_C`` are all IDLE. 
2. When the parent PipelineSequence is first ticked, let's assume ``Action_A`` returns ``RUNNING``. The parent node will now return ``RUNNING`` and no other nodes are ticked.

|

 .. image:: images/custom_behavior_tree/control_pipelineSequence_RUNNING_IDLE_IDLE.png
    :align: center

| 

3. Now, let's assume ``Action_A`` returns ``SUCCESS``, ``Action_B`` will now get ticked and will return ``RUNNING``. ``Action_C`` has not yet been ticked so will return ``IDLE``.

|

 .. image:: images/custom_behavior_tree/control_pipelineSequence_SUCCESS_RUNNING_IDLE.png
    :align: center

| 

4. ``Action_A`` gets ticked again and returns ``RUNNING``, and ``Action_B`` gets re-ticked and returns ``SUCCESS`` and therefore the BT goes on to tick ``Action_C`` for the first time. Let's assume ``Action_C`` returns ``RUNNING``. The retick-ing of ``Action_A`` is what makes PipelineSequence useful.

|

 .. image:: images/custom_behavior_tree/control_pipelineSequence_RUNNING_SUCCESS_RUNNING.png
    :align: center

| 

5. All actions in the sequence will be re-ticked. Let's assume ``Action_A`` still returns ``RUNNING``, where as ``Action_B`` returns ``SUCCESS`` again, and ``Action_C`` now returns ``SUCCESS`` on this tick. The sequence is now complete, and therefore ``Action_A`` is halted, even though it was still ``RUNNING``.

|

 .. image:: images/custom_behavior_tree/control_pipelineSequence_RUNNING_SUCCESS_SUCCESS.png
    :align: center

| 

Recall that if ``Action_A``, ``Action_B``, or ``Action_C`` returned ``FAILURE`` at any point  of time, the parent would have returned ``FAILURE`` and halted any children as well.

For additional details regarding the ``PipelineSequence`` please see the `PipelineSequence configuration guide <../../configuration/packages/bt-plugins/controls/PipelineSequence.html>`_.

Control: Recovery
---------------------
The Recovery control node has only two children and returns ``SUCCESS`` if and only if the first child returns ``SUCCESS``. 
If the first child returns ``FAILURE``, the second child will be ticked. This loop will continue until either:

- The first child returns ``SUCCESS`` (which results in ``SUCCESS`` of the parent node)

- The second child returns ``FAILURE`` (which results in ``FAILURE`` of the parent node)

- The ``number_of_retries`` input parameter is violated

This node is usually used to link together an action, and a recovery action as the name suggests. The first action will typically be the "main" behavior,
and the second action will be something to be done in case of ``FAILURE`` of the main behavior. Often, the ticking of the second child action will promote the chance the first action will succeed.

|

 .. image:: images/custom_behavior_tree/control_recovery_node.png
    :align: center

| 

.. code-block:: xml

    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <RecoveryNode number_of_retries="1">
                <ComputePathToPose/>
                <ClearLocalCostmap/>
            </RecoveryNode>
        </BehaviorTree>
    </root>

In the above example, let's assume ``ComputePathToPose`` fails. ``ClearLocalCostmap`` will be ticked in response, and return ``SUCCESS``.
Now that we have cleared the costmap, lets' say the robot is correctly able to compute the path and ``ComputePathToPose`` now returns ``SUCCESS``. Then, the parent RecoveryNode will also return ``SUCCESS`` and the BT will be complete.

For additional details regarding the ``RecoveryNode`` please see the `RecoveryNode configuration guide <../../configuration/packages/bt-plugins/controls/RecoveryNode.html>`_.

Control: RoundRobin
-----------------------
The RoundRobin control node ticks it's children in a round robin fashion until a child returns ``SUCCESS``, in which the parent node will also return ``SUCCESS``. 
If all children return ``FAILURE`` so will the parent RoundRobin.

Here is an example BT we will use to walk through the concept.

|

 .. image:: images/custom_behavior_tree/control_round_robin.png
    :align: center

|                  

.. code-block:: xml

    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <RoundRobin>
                <Action_A/>
                <Action_B/>
                <Action_C/>
            </RoundRobin>
        </BehaviorTree>
    </root>

1. All the nodes start at ``IDLE``

|

 .. image:: images/custom_behavior_tree/control_round_robin_IDLE_IDLE_IDLE.png
    :align: center

| 

2. Upon tick of the parent node, the first child (``Action_A`` is ticked. Let's assume on tick the child returns ``RUNNING``.
In this case, no other children are ticked and the parent node returns ``RUNNING`` as well.

|

 .. image:: images/custom_behavior_tree/control_round_robin_RUNNING_IDLE_IDLE.png
    :align: center

| 

3. Upon the next tick, let's assume that ``Action_A`` returns ``FAILURE``. 
This means that ``Action_B`` will get ticked next, and ``Action_C`` remains unticked. 
Let's assume ``Action_B`` returns ``RUNNING`` this time. That means the parent RoundRobin node will also return ``RUNNING``.

|

 .. image:: images/custom_behavior_tree/control_round_robin_FAILURE_RUNNING_IDLE.png
    :align: center

| 

4. Upon this next tick,  let's assume that ``Action_B`` returns ``SUCCESS``. The parent RoundRobin will now halt all children and returns ``SUCCESS``. 
The parent node retains this state information, and will tick ``Action_C`` upon the next tick rather than start from ``Action_A`` like Step 2 did.

|

 .. image:: images/custom_behavior_tree/control_round_robin_FAILURE_SUCCESS_IDLE.png
    :align: center

| 

5. On this tick, let's assume ``Action_C`` returns ``RUNNING``, and so does the parent RoundRobin. No other nodes are ticked.

|

 .. image:: images/custom_behavior_tree/control_round_robin_FAILURE_SUCCESS_RUNNING.png
    :align: center

| 

   
6. On this last tick, let's assume ``Action_C`` returns ``FAILURE``. The parent will circle and tick ``Action_A`` again. ``Action_A`` returns ``RUNNING`` and so will the parent RoundRobin node. This pattern will continue indefinitely.

|

 .. image:: images/custom_behavior_tree/control_round_robin_RUNNING_IDLE_FAILURE.png
    :align: center

| 

For additional details regarding the ``RecoveryNode`` please see the `RoundRobin configuration guide <../../configuration/packages/bt-plugins/controls/RoundRobin.html>`_.

Navigate With Replanning and Recovery
=====================================

The following section will describe in detail the concept of the main and default BT currently used in Nav2, ``navigate_w_replanning_and_recovery.xml``.
This behavior tree replans the global path periodically at 1 Hz and it also has recovery actions.

|

 .. image:: images/custom_behavior_tree/overall_bt.png
    :align: center

|                  

BTs are primarily defined in XML. The tree shown above is represented in XML as follows.

.. code-block:: xml

    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <RecoveryNode number_of_retries="6" name="NavigateRecovery">
                <PipelineSequence name="NavigateWithReplanning">
                    <RateController hz="1.0">
                        <RecoveryNode number_of_retries="1" name="ComputePathToPose">
                            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                            <ReactiveFallback name="ComputePathToPoseRecoveryFallback">
                                <GoalUpdated/>
                                <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
                            </ReactiveFallback>
                        </RecoveryNode>
                    </RateController>
                    <RecoveryNode number_of_retries="1" name="FollowPath">
                        <FollowPath path="{path}" controller_id="FollowPath"/>
                        <ReactiveFallback name="FollowPathRecoveryFallback">
                            <GoalUpdated/>
                            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
                        </ReactiveFallback>
                    </RecoveryNode>
                </PipelineSequence>
                <ReactiveFallback name="RecoveryFallback">
                    <GoalUpdated/>
                    <RoundRobin name="RecoveryActions">
                        <Sequence name="ClearingActions">
                            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
                            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
                        </Sequence>
                        <Spin spin_dist="1.57"/>
                        <Wait wait_duration="5"/>
                        <BackUp backup_dist="0.15" backup_speed="0.025"/>
                    </RoundRobin>
                </ReactiveFallback>
            </RecoveryNode>
        </BehaviorTree>
    </root>
                                                                                                                

This is likely still a bit overwhelming, but this tree can be broken into two smaller subtrees that we can focus on one at a time.
These smaller subtrees are the children of the top-most ``RecoveryNode``. From this point forward the ``NavigateWithReplanning`` subtree will be referred to as the ``Navigation`` subtree, and the ``RecoveryFallback`` subtree will be known as the ``Recovery`` subtree.
This can be represented in the following way:

|

 .. image:: images/custom_behavior_tree/overall_bt_w_breakdown.png
    :align: center

|          

The ``Navigation`` subtree mainly involves actual navigation behavior:

- calculating a path
  
- following a path
  
- contextual recovery behaviors for each of the above primary navigation behaviors
  
The ``Recovery`` subtree includes recovery behaviors for system level failures or items that were not easily dealt with internally.

The overall BT will (hopefully) spend most of its time in the ``Navigation`` subtree. If either of the two main behaviors in the ``Navigation`` subtree fail
(path calculation or path following), contextual recoveries will be attempted.

If the contextual recoveries were still not enough, the ``Navigation`` subtree will return ``FAILURE``. 
The system will move on to the ``Recovery`` subtree to attempt to clear any system level navigation failures.

This happens until the ``number_of_retries`` for the parent ``RecoveryNode`` is exceeded (which by default is 6).

.. code-block:: xml

    <RecoveryNode number_of_retries="6" name="NavigateRecovery">

Navigation Subtree
======================

Now that we have gone over the control flow between the ``Navigation`` subtree and the ``Recovery`` subtree, 
let's focus on the Navigation subtree.
|

 .. image:: images/custom_behavior_tree/navigation_subtree.png
    :align: center

|         

The XML of this subtree is as follows:

.. code-block:: xml

    <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
            <RecoveryNode number_of_retries="1" name="ComputePathToPose">
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                <ReactiveFallback name="ComputePathToPoseRecoveryFallback">
                    <GoalUpdated/>
                    <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
                </ReactiveFallback>
            </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
            <FollowPath path="{path}" controller_id="FollowPath"/>
            <ReactiveFallback name="FollowPathRecoveryFallback">
                <GoalUpdated/>
                <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
            </ReactiveFallback>
        </RecoveryNode>
    </PipelineSequence>
                                 
This subtree has two primary actions ``ComputePathToPose`` and ``FollowPath``.
If either of these two actions fail, they will attempt to clear the failure contextually. 
The crux of the tree can be represented with only one parent and two children nodes like this:

|

 .. image:: images/custom_behavior_tree/navigation_subtree_bare.png
    :align: center

|       

The parent ``PipelineSequence`` node allows the ``ComputePathToPose`` to be ticked, and once that succeeds, ``FollowPath`` to be ticked. 
While the ``FollowPath`` subtree is being ticked, the ``ComputePathToPose`` subtree will be ticked as well. This allows for the path to be recomputed as the robot moves around. 

Both the ``ComputePathToPose`` and the ``FollowPath`` follow the same general structure.

- Do the action

- If the action fails, try to see if we can contextually recover

The below is the ``ComputePathToPose`` subtree:

|

 .. image:: images/custom_behavior_tree/contextual_recoveries.png
    :align: center

|      

The parent ``RecoveryNode`` controls the flow between the action, and the contextual recovery subtree. 
The contextual recoveries for both ``ComputePathToPose`` and ``FollowPath`` involve checking if the goal has been updated, and involves clearing the relevant costmap.

Consider changing the ``number_of_retries`` parameter in the parent ``RecoveryNode`` control node if your application can tolerate more attempts at contextual recoveries before moving on to system-level recoveries.

The only differences in the BT subtree of ``ComputePathToPose`` and ``FollowPath`` are outlined below:

- The action node in the subtree:
    - The ``ComputePathToPose`` subtree centers around the ``ComputePathToPose`` action. 
    - The ``FollowPath`` subtree centers around the ``FollowPath`` action.

- The ``RateController`` that decorates the ``ComputePathToPose`` subtree
    The ``RateController`` decorates the ``ComputePathToPose`` subtree to keep planning at the specified frequency. The default frequency for this BT is 1 hz. 
    This is done to prevent the BT from flooding the planning server with too many useless requests at the tree update rate (100Hz). Consider changing this frequency to something higher or lower depending on the application and the computational cost of 
    calculating the path. 
    
- The costmap that is being cleared within the contextual recovery:
    - The ``ComputePathToPose`` subtree clears the global costmap. The global costmap is the relevant costmap in the context of the planner
    - The ``FollowPath`` subtree clears the local costmap. The local costmap is the relevant costmap in the context of the controller

Recovery Subtree
================
The ``Recovery`` subtree is the second big "half" of the Nav2 default ``navigate_w_replanning_and_recovery.xml`` tree.
In short, this subtree is triggered when the ``Navigation`` subtree returns ``FAILURE`` controls the recoveries at the system level (in the case the contextual recoveries in the ``Navigation`` subtree were not sufficient).
                               
|

 .. image:: images/custom_behavior_tree/recovery_subtree.png
    :align: center

|         

And the XML snippet:

.. code-block:: xml

    <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
            <Sequence name="ClearingActions">
                <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
                <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
            <Spin spin_dist="1.57"/>
            <Wait wait_duration="5"/>
            <BackUp backup_dist="0.15" backup_speed="0.025"/>
        </RoundRobin>
    </ReactiveFallback>

The top most parent, ``ReactiveFallback`` controls the flow between the rest of the system wide recoveries, and asynchronously checking if a new goal has been received.
If at any point the goal gets updated, this subtree will halt all children and return ``SUCCESS``.
This should look familiar to the contextual recovery portions of the ``Navigation`` subtree. This is a common BT pattern to handle the situation "Unless 'this condition' happens, Do action A".

These condition nodes can be extremely powerful and are typically paired with ``ReactiveFallback``. It can be easy to imagine wrapping this whole ``navigate_w_replanning_and_recovery`` tree
in a ``ReactiveFallback`` with a ``isBatteryLow`` condition -- meaning the ``navigate_w_replanning_and_recovery`` tree will execute *unless* the battery becomes low (and then entire a different subtree for docking to recharge). 

If the goal is never updated, the behavior tree will go on to the ``RoundRobin`` node. These are the default four system-level recoveries in the BT are:

- A sequence that clears both costmaps (local, and global)

- ``Spin`` recovery action

- ``Wait`` recovery action

- ``BackUp`` recovery action

Upon ``SUCCESS`` of any of the four children of the parent ``RoundRobin``, the robot will attempt to renavigate in the ``Navigation`` subtree. 
If this renavigation was not successful, the next child of the ``RoundRobin`` will be ticked.

For example, let's say the robot is stuck and the ``Navigation`` subtree returns ``FAILURE``:
(for the sake of this example, let's assume that the goal is never updated).

1. The Costmap clearing sequence in the ``Recovery`` subtree is attempted, and returns ``SUCCESS``. The robot now moves to ``Navigation`` subtree again

2. Let's assume that clearing both costmaps was not sufficient, and the ``Navigation`` subtree returns ``FAILURE`` once again. The robot now ticks the ``Recovery`` subtree

3. In the ``Recovery`` subtree, the ``Spin`` action will be ticked. If this returns ``SUCCESS``, then the robot will return to the main ``Navigation`` subtree *BUT* let's assume that the ``Spin`` recovery returns ``FAILURE``. In this case, the tree will *remain* in the ``Recovery`` subtree

4. Let's say the next recovery action, ``Wait`` returns ``SUCCESS``. The robot will then move on to the ``Navigation`` subtree

5. Assume  the ``Navigation`` subtree returns ``FAILURE`` (clearing the costmaps, attempting a spin, and waiting were *still* not sufficient to recover the system. The robot will move onto the ``Recovery`` subtree and attempt the ``BackUp`` action

The above logic will go on indefinitely until the ``number_of_retries`` in the parent of the ``Navigate`` subtree and ``Recovery`` subtree is exceeded, or if all the system-wide recoveries in the ``Recovery`` subtree return ``FAILURE`` (this is unlikely, and likely points to some other system failure).

Custom Action
=============

Adding to Launch File
=====================

Testing
=======
