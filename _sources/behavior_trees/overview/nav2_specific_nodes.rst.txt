.. _nav2_specific_nodes:

Introduction To Nav2 Specific Nodes
===================================
.. warning::
    Vocabulary can be a large point of confusion here when first starting out.
        - A ``Node`` when discussing BTs is entirely different than a ``Node`` in the ROS 2 context

        - An ``ActionNode`` in the context of BTs is not necessarily connected to an Action Server in the ROS 2 context (but often it is)

There are quite a few custom Nav2 BT nodes that are provided to be used in the Nav2 specific fashion. Some commonly used Nav2 nodes will be described below.
The full list of custom BT nodes can be found in the `nav2_behavior_tree plugins folder <https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree/plugins>`_.
The `configuration guide <../../configuration/packages/configuring-bt-xml.html>`_ can also be quite useful.

Action Nodes
------------

* ComputePathToPose - ComputePathToPose Action Server Client (Planner Interface)

* FollowPath - FollowPath Action Server Client (Controller Interface)

* Spin, Wait, Backup - Behaviors Action Server Client

* ClearCostmapService - ClearCostmapService Server Clients

Upon completion, these action nodes will return ``SUCCESS`` if the action server believes the action has been completed correctly, ``RUNNING`` when still running, and will return ``FAILURE`` otherwise. Note that in the above list,
the `ClearCostmapService` action node is *not* an action server client, but a service client.

Condition Nodes
---------------

* GoalUpdated - Checks if the goal on the goal topic has been updated

* GoalReached - Checks if the goal has been reached

* InitialPoseReceived - Checks to see if a pose on the ``initial_pose`` topic has been received

* isBatteryLow - Checks to see if the battery is low by listening on the battery topic

The above list of condition nodes can be used to probe particular aspects of the system. Typically they will return ``SUCCESS`` if the condition is true and ``FAILURE`` otherwise.
The key condition that is used in the default Nav2 BT is ``GoalUpdated`` which is checked asynchronously within particular subtrees. This condition node allows for the behavior described as "If the goal has been updated, then we must replan".
Condition nodes are typically paired with ReactiveFallback nodes.

Decorator Nodes
---------------

* Distance Controller - Will tick children nodes every time the robot has traveled a certain distance

* Rate Controller - Controls the ticking of its child node at a constant frequency. The tick rate is an exposed port

* Goal Updater - Will update the goal of children nodes via ports on the BT

* Single Trigger - Will only tick its child node once, and will return ``FAILURE`` for all subsequent ticks

* Speed Controller - Controls the ticking of its child node at a rate proportional to the robot's speed

Control: PipelineSequence
-------------------------
The ``PipelineSequence`` control node re-ticks previous children when a child returns ``RUNNING``.
This node is similar to the ``Sequence`` node, with the additional property that the children prior to the "current" are re-ticked, (resembling the flow of water in a pipe).
If at any point a child returns ``FAILURE``, all children will be halted and the parent node will also return ``FAILURE``. Upon ``SUCCESS`` of the **last node** in the sequence, this node will halt and return ``SUCCESS``.

To explain this further, here is an example BT that uses PipelineSequence.

|

 .. image:: ../images/control_pipelineSequence.png
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

1. ``Action_A``, ``Action_B``, and ``Action_C`` are all ``IDLE``.
2. When the parent PipelineSequence is first ticked, let's assume ``Action_A`` returns ``RUNNING``. The parent node will now return ``RUNNING`` and no other nodes are ticked.

|

 .. image:: ../images/control_pipelineSequence_RUNNING_IDLE_IDLE.png
    :align: center

|

3. Now, let's assume ``Action_A`` returns ``SUCCESS``, ``Action_B`` will now get ticked and will return ``RUNNING``. ``Action_C`` has not yet been ticked so will return ``IDLE``.

|

 .. image:: ../images/control_pipelineSequence_SUCCESS_RUNNING_IDLE.png
    :align: center

|

4. ``Action_A`` gets ticked again and returns ``RUNNING``, and ``Action_B`` gets re-ticked and returns ``SUCCESS`` and therefore the BT goes on to tick ``Action_C`` for the first time. Let's assume ``Action_C`` returns ``RUNNING``. The retick-ing of ``Action_A`` is what makes PipelineSequence useful.

|

 .. image:: ../images/control_pipelineSequence_RUNNING_SUCCESS_RUNNING.png
    :align: center

|

5. All actions in the sequence will be re-ticked. Let's assume ``Action_A`` still returns ``RUNNING``, where as ``Action_B`` returns ``SUCCESS`` again, and ``Action_C`` now returns ``SUCCESS`` on this tick. The sequence is now complete, and therefore ``Action_A`` is halted, even though it was still ``RUNNING``.

|

 .. image:: ../images/control_pipelineSequence_RUNNING_SUCCESS_SUCCESS.png
    :align: center

|

Recall that if ``Action_A``, ``Action_B``, or ``Action_C`` returned ``FAILURE`` at any point of time, the parent would have returned ``FAILURE`` and halted any children as well.

For additional details regarding the ``PipelineSequence`` please see the `PipelineSequence configuration guide <../../configuration/packages/bt-plugins/controls/PipelineSequence.html>`_.

Control: Recovery
---------------------
The Recovery control node has only two children and returns ``SUCCESS`` if and only if the first child returns ``SUCCESS``.
If the first child returns ``FAILURE``, the second child will be ticked. This loop will continue until either:

* The first child returns ``SUCCESS`` (which results in ``SUCCESS`` of the parent node)

* The second child returns ``FAILURE`` (which results in ``FAILURE`` of the parent node)

* The ``number_of_retries`` input parameter is violated

This node is usually used to link together an action, and a recovery action as the name suggests. The first action will typically be the "main" behavior,
and the second action will be something to be done in case of ``FAILURE`` of the main behavior. Often, the ticking of the second child action will promote the chance the first action will succeed.

|

 .. image:: ../images/control_recovery_node.png
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
Now that we have cleared the costmap, let's say the robot is correctly able to compute the path and ``ComputePathToPose`` now returns ``SUCCESS``. Then, the parent RecoveryNode will also return ``SUCCESS`` and the BT will be complete.

For additional details regarding the ``RecoveryNode`` please see the `RecoveryNode configuration guide <../../configuration/packages/bt-plugins/controls/RecoveryNode.html>`_.

Control: RoundRobin
-----------------------
The RoundRobin control node ticks its children in a round robin fashion until a child returns ``SUCCESS``, in which the parent node will also return ``SUCCESS``.
If all children return ``FAILURE`` so will the parent RoundRobin.

Here is an example BT we will use to walk through the concept.

|

 .. image:: ../images/control_round_robin.png
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

 .. image:: ../images/control_round_robin_IDLE_IDLE_IDLE.png
    :align: center

|

2. Upon tick of the parent node, the first child (``Action_A``) is ticked. Let's assume on tick the child returns ``RUNNING``.
In this case, no other children are ticked and the parent node returns ``RUNNING`` as well.

|

 .. image:: ../images/control_round_robin_RUNNING_IDLE_IDLE.png
    :align: center

|

3. Upon the next tick, let's assume that ``Action_A`` returns ``FAILURE``.
This means that ``Action_B`` will get ticked next, and ``Action_C`` remains unticked.
Let's assume ``Action_B`` returns ``RUNNING`` this time. That means the parent RoundRobin node will also return ``RUNNING``.

|

 .. image:: ../images/control_round_robin_FAILURE_RUNNING_IDLE.png
    :align: center

|

4. Upon this next tick,  let's assume that ``Action_B`` returns ``SUCCESS``. The parent RoundRobin will now halt all children and return ``SUCCESS``.
The parent node retains this state information, and will tick ``Action_C`` upon the next tick rather than start from ``Action_A`` like Step 2 did.

|

 .. image:: ../images/control_round_robin_FAILURE_SUCCESS_IDLE.png
    :align: center

|

5. On this tick, let's assume ``Action_C`` returns ``RUNNING``, and so does the parent RoundRobin. No other nodes are ticked.

|

 .. image:: ../images/control_round_robin_FAILURE_SUCCESS_RUNNING.png
    :align: center

|


6. On this last tick, let's assume ``Action_C`` returns ``FAILURE``. The parent will circle and tick ``Action_A`` again. ``Action_A`` returns ``RUNNING`` and so will the parent RoundRobin node. This pattern will continue indefinitely unless all children return ``FAILURE``.

|

 .. image:: ../images/control_round_robin_RUNNING_IDLE_FAILURE.png
    :align: center

|

For additional details regarding the ``RoundRobin`` please see the `RoundRobin configuration guide <../../configuration/packages/bt-plugins/controls/RoundRobin.html>`_.

Control: NonblockingSequence
----------------------------

The ``NonblockingSequence`` control node ticks all children as long as they return ``SUCCESS`` or  ``RUNNING``. This node is similar to the ``PipelineSequence`` node, with the additional property that all children are re-ticked as long as ``SUCCESS`` or ``RUNNING``, instead of stopping at the latest ``RUNNING`` node. If at any point a child returns ``FAILURE``, all children will be halted and the parent node will also return ``FAILURE``. Upon ``SUCCESS`` of **all nodes** in the sequence, this node will halt and return ``SUCCESS``.

Note that even if a node returns ``SUCCESS`` in a previous tick, on the next tick, the ``NonblockingSequence`` will still tick the successful node, possibly restarting it. This is to ensure that successful nodes do not latch a stale state while waiting for another long running node to be complete

To explain this further, here is an example BT that uses NonblockingSequence.

|

 .. image:: ../images/control_nonblockingSequence.png
    :align: center



.. code-block:: xml

    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <NonblockingSequence>
                <Action_A/>
                <Action_B/>
                <Action_C/>
            </NonblockingSequence>
        </BehaviorTree>
    </root>

1. ``Action_A``, ``Action_B``, and ``Action_C`` are all ``IDLE``.

2. When the parent NonblockingSequence is first ticked, let's assume ``Action_A`` returns ``RUNNING``. Following this, ``Action_B`` will be ticked, and let's assume it also returns ``RUNNING``. Finally, ``Action_C`` will be ticked, and let's assume it also returns ``RUNNING``. With three ``RUNNING`` children, the NonblockingSequence will return ``RUNNING``

|

 .. image:: ../images/control_nonblockingSequence_RUNNING_RUNNING_RUNNING.png
    :align: center

|

3. On the next tick of the the parent NonblockingSequence, all actions in the sequence will be re-ticked. Let's assume ``Action_A`` returns ``SUCCESS``, and ``Action_B`` and ``Action_C`` still return ``RUNNING``. In this configuration, the NonblockingSequence still returns ``RUNNING``, as there are two nodes in the children that are ``RUNNING``

|

 .. image:: ../images/control_nonblockingSequence_SUCCESS_RUNNING_RUNNING.png
    :align: center

|

4. Now, let's assume on the next re-tick, ``Action_A`` and ``Action_C``  return ``SUCCESS``, and ``Action_B`` returns ``RUNNING``. In this configuration, the NonblockingSequence still returns ``RUNNING``, as there is still one child node that is ``RUNNING``. Note that ``ActionA`` was re-ticked and again returned ``SUCCESS`` in this case, it did not skip due to previously returning `SUCCESS``.

|

 .. image:: ../images/control_nonblockingSequence_SUCCESS_RUNNING_SUCCESS.png
    :align: center

|

5. Finally, Let's assume ``Action_A``, ``Action_B``, and ``Action_C``  all return ``SUCCESS``. The sequence is now complete, and therefore ``Action_A``, ``Action_B``, and ``Action_C`` are all halted and NonblockingSequence returns ``SUCCESS``.

|

 .. image:: ../images/control_nonblockingSequence_SUCCESS_SUCCESS_SUCCESS.png
    :align: center

|

Recall that if ``Action_A``, ``Action_B``, or ``Action_C`` returned ``FAILURE`` at any point of time, the parent would have returned ``FAILURE`` and halted any children as well.

For additional details regarding the ``NonblockingSequence`` please see the `NonblockingSequence configuration guide <../../configuration/packages/bt-plugins/controls/NonblockingSequence.html>`_.

Control: PersistentSequence
----------------------------

The ``PersistentSequence`` is similar to the ``Sequence`` node, but it stores the index of the last running child in the blackboard (key: "current_child_idx"), and it does not reset the index on halt.

For more information see the ``Sequence`` BT node in BT.CPP.

.. code-block:: xml

    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <Script code="current_child_idx := 0" />
            <PersistentSequence current_child_idx="{current_child_idx}">
                <Action_A/>
                <Action_B/>
                <Action_C/>
            </PersistentSequence>
        </BehaviorTree>
    </root>

Control: PauseResumeController
------------------------------

The ``PauseResumeController`` is a control node that adds pause and resume functionality to a behavior tree through service calls.

It has one mandatory child for the RESUMED, and three optional for the PAUSED state, the ON_PAUSE event and the ON_RESUME event.
It has two input ports:

- ``pause_service_name``: name of the service to pause
- ``resume_service_name``: name of the service to resume

1. The controller starts in RESUMED state, and ticks it until it returns success.
2. When the pause service is called, ON_PAUSE is ticked until completion, then the controller switches to PAUSED state.
3. In PAUSED state the PAUSED child is ticked until the state is changed, or until it returns failure.
4. When the resume service is called, ON_RESUME is ticked until completion, then the controller switches back to RESUMED state.

The controller only returns success when the RESUMED child returns success. The controller returns failure if any child returns failure. In any other case, it returns running.

.. code-block:: xml

    <PauseResumeController pause_service_name="/pause" resume_service_name="/resume">
        <!-- RESUMED branch -->

        <!-- PAUSED branch (optional) -->

        <!-- ON_PAUSE branch (optional) -->

        <!-- ON_RESUME branch (optional) -->
    </PauseResumeController>

When the ON_PAUSE and ON_RESUME branches fail, the controller will return failure, halt, and the state will be reset to RESUMED. It might be desirable to retry the transition a few times before failing for real, which functionality is not built in the controller node, but is easily achievable by adding a retry node in the BT:

.. code-block:: xml

    <PauseResumeController pause_service_name="/pause" resume_service_name="/resume">
        <!-- RESUMED branch -->

        <!-- PAUSED branch -->

        <RetryUntilSuccessful num_attempts="3">
            <!-- ON_PAUSE branch -->
        </RetryUntilSuccessful>

        <RetryUntilSuccessful num_attempts="3">
            <!-- ON_RESUME branch -->
        </RetryUntilSuccessful>
    </PauseResumeController>
