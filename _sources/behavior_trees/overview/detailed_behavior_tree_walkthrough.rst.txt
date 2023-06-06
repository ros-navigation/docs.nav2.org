.. _detailed_behavior_tree_walkthrough:

Detailed Behavior Tree Walkthrough
**********************************

- `Overview`_
- `Prerequisites`_
- `Navigate To Pose With Replanning and Recovery`_
- `Navigation Subtree`_
- `Recovery Subtree`_

Overview
========

This document serves as a reference guide to the main behavior tree (BT) used in Nav2.

There are many example behavior trees provided in ``nav2_bt_navigator/behavior_trees``,
but these sometimes have to be re-configured based on the application of the robot.
The following document will walk through the current main default BT ``navigate_to_pose_w_replanning_and_recovery.xml``
in great detail.

Prerequisites
=============

- Become familiar with the concept of a behavior tree before continuing with this walkthrough

    - Read the short explanation in `navigation concepts <../../concepts/index.html>`_

    - Read the general tutorial and guide (not Nav2 specific) on the `BehaviorTree CPP V3 <https://www.behaviortree.dev/>`_ website. Specifically, the "Learn the Basics" section on the BehaviorTree CPP V3 website explains the basic generic nodes that will be used that this guide will build upon.

- Become familiar with the custom `Nav2 specific BT nodes <nav2_specific_nodes.html>`_

Navigate To Pose With Replanning and Recovery
=============================================

The following section will describe in detail the concept of the main and default BT currently used in Nav2, ``navigate_to_pose_w_replanning_and_recovery.xml``.
This behavior tree replans the global path periodically at 1 Hz and it also has recovery actions.

|

 .. image:: ../images/walkthrough/overall_bt.png
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

 .. image:: ../images/walkthrough/overall_bt_w_breakdown.png
    :align: center

|

The ``Navigation`` subtree mainly involves actual navigation behavior:

- calculating a path

- following a path

- contextual recovery behaviors for each of the above primary navigation behaviors

The ``Recovery`` subtree includes behaviors for system level failures or items that were not easily dealt with internally.

The overall BT will (hopefully) spend most of its time in the ``Navigation`` subtree. If either of the two main behaviors in the ``Navigation`` subtree fail
(path calculation or path following), contextual recoveries will be attempted.

If the contextual recoveries were still not enough, the ``Navigation`` subtree will return ``FAILURE``.
The system will move on to the ``Recovery`` subtree to attempt to clear any system level navigation failures.

This happens until the ``number_of_retries`` for the parent ``RecoveryNode`` is exceeded (which by default is 6).

.. code-block:: xml

    <RecoveryNode number_of_retries="6" name="NavigateRecovery">

Navigation Subtree
======================

Now that we have gone over the control flow between the ``Navigation`` subtree and the ``Recovery`` subtree, let's focus on the Navigation subtree.

|

 .. image:: ../images/walkthrough/navigation_subtree.png
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

 .. image:: ../images/walkthrough/navigation_subtree_bare.png
    :align: center

|

The parent ``PipelineSequence`` node allows the ``ComputePathToPose`` to be ticked, and once that succeeds, ``FollowPath`` to be ticked.
While the ``FollowPath`` subtree is being ticked, the ``ComputePathToPose`` subtree will be ticked as well. This allows for the path to be recomputed as the robot moves around.

Both the ``ComputePathToPose`` and the ``FollowPath`` follow the same general structure.

- Do the action

- If the action fails, try to see if we can contextually recover

The below is the ``ComputePathToPose`` subtree:

|

 .. image:: ../images/walkthrough/contextual_recoveries.png
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
    calculating the path. There are other decorators that can be used instead of the ``RateController``. Consider using the ``SpeedController`` or ``DistanceController`` decorators if appropriate.

- The costmap that is being cleared within the contextual recovery:
    - The ``ComputePathToPose`` subtree clears the global costmap. The global costmap is the relevant costmap in the context of the planner
    - The ``FollowPath`` subtree clears the local costmap. The local costmap is the relevant costmap in the context of the controller

Recovery Subtree
================

The ``Recovery`` subtree is the second big "half" of the Nav2 default ``navigate_to_pose_w_replanning_and_recovery.xml`` tree.
In short, this subtree is triggered when the ``Navigation`` subtree returns ``FAILURE`` and controls the recoveries at the system level (in the case the contextual recoveries in the ``Navigation`` subtree were not sufficient).

|

 .. image:: ../images/walkthrough/recovery_subtree.png
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

The top most parent, ``ReactiveFallback`` controls the flow between the rest of the system wide recoveries, and asynchronously checks if a new goal has been received.
If at any point the goal gets updated, this subtree will halt all children and return ``SUCCESS``. This allows for quick reactions to new goals and preempt currently executing recoveries.
This should look familiar to the contextual recovery portions of the ``Navigation`` subtree. This is a common BT pattern to handle the situation "Unless 'this condition' happens, Do action A".

These condition nodes can be extremely powerful and are typically paired with ``ReactiveFallback``. It can be easy to imagine wrapping this whole ``navigate_to_pose_w_replanning_and_recovery`` tree
in a ``ReactiveFallback`` with a ``isBatteryLow`` condition -- meaning the ``navigate_to_pose_w_replanning_and_recovery`` tree will execute *unless* the battery becomes low (and then entire a different subtree for docking to recharge).

If the goal is never updated, the behavior tree will go on to the ``RoundRobin`` node. These are the default four system-level recoveries in the BT are:

- A sequence that clears both costmaps (local, and global)

- ``Spin`` action

- ``Wait`` action

- ``BackUp`` action

Upon ``SUCCESS`` of any of the four children of the parent ``RoundRobin``, the robot will attempt to renavigate in the ``Navigation`` subtree.
If this renavigation was not successful, the next child of the ``RoundRobin`` will be ticked.

For example, let's say the robot is stuck and the ``Navigation`` subtree returns ``FAILURE``:
(for the sake of this example, let's assume that the goal is never updated).

1. The Costmap clearing sequence in the ``Recovery`` subtree is attempted, and returns ``SUCCESS``. The robot now moves to ``Navigation`` subtree again

2. Let's assume that clearing both costmaps was not sufficient, and the ``Navigation`` subtree returns ``FAILURE`` once again. The robot now ticks the ``Recovery`` subtree

3. In the ``Recovery`` subtree, the ``Spin`` action will be ticked. If this returns ``SUCCESS``, then the robot will return to the main ``Navigation`` subtree *BUT* let's assume that the ``Spin`` action returns ``FAILURE``. In this case, the tree will *remain* in the ``Recovery`` subtree

4. Let's say the next action, ``Wait`` returns ``SUCCESS``. The robot will then move on to the ``Navigation`` subtree

5. Assume  the ``Navigation`` subtree returns ``FAILURE`` (clearing the costmaps, attempting a spin, and waiting were *still* not sufficient to recover the system. The robot will move onto the ``Recovery`` subtree and attempt the ``BackUp`` action. Let's say that the robot attempts the ``BackUp`` action and was able to successfully complete the action. The ``BackUp`` action node returns ``SUCCESS`` and so now we move on to the Navigation subtree again.

6. In this hypothetical scenario, let's assume that the ``BackUp`` action allowed the robot to successfully navigate in the ``Navigation`` subtree, and the robot reaches the goal. In this case, the overall BT will still return ``SUCCESS``.

If the ``BackUp`` action was not sufficient enough to allow the robot to become un-stuck, the above logic will go on indefinitely until the ``number_of_retries`` in the parent of the ``Navigate`` subtree and ``Recovery`` subtree is exceeded, or if all the system-wide recoveries in the ``Recovery`` subtree return ``FAILURE`` (this is unlikely, and likely points to some other system failure).



