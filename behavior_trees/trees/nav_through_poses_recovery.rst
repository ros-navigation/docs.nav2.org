.. _behavior_tree_nav_through_poses:

Navigate Through Poses
######################

This behavior tree implements navigation from a starting point, through
intermediary hard pose constraints, to a final goal in freespace.
It contains both use of custom behaviors for recovery in specific sub-contexts as well as a global recovery subtree for system-level failures.
It also provides the opportunity for users to retry tasks multiple times before returning a failed state.

Prerequisites
*************

- Understand BehaviorTree `fallback control <https://www.behaviortree.dev/docs/nodes-library/fallbacknode/>`_.
- Understand BehaviorTree `reactive behavior <https://www.behaviortree.dev/docs/tutorial-basics/tutorial_04_sequence/>`_.
- Understand `Nav2 Specific Nodes <https://docs.nav2.org/behavior_trees/overview/nav2_specific_nodes.html>`_ (PipelineSequence, Recovery, RoundRobin).

Detailed Explanations
*********************

The ``ComputePathThroughPoses`` and ``FollowPath`` BT nodes both also specify their algorithms to utilize.
By convention we name these by their role (e.g. ``FollowPath`` instead of ``DWB``)
such that a behavior tree or application developer need not worry about the technical specifics.
They just want to use a path following controller.

In this behavior tree, the full navigation task is retried up to 6 times before
returning failure to the caller. This gives the system ample time and opportunity to try to
recovery from failure conditions or wait for transient issues to pass,
such as crowding from people or a temporary sensor failure.

NavigateWithReplanning PipelineSequence -- Main Branch
======================================================

In nominal execution, ``PipelineSequence`` keeps control active while the
planner side is evaluated at ``0.333`` Hz (about every 3 seconds), similar to
:ref:`behavior_trees`.

The selector nodes (progress checker, goal checker, path handler, controller,
planner) are ticked first so plugins are configured before motion continues.

Once an initial valid path is found by ``FallbackComputePathToPose`` planner branch,
``FollowPath`` controller branch is ticked while the planner branch
continues running, so planning and control run in a pipeline.

The pipeline continues while branches return ``RUNNING`` and none returns
``FAILURE``. If all required branches return ``SUCCESS``, navigation succeeds.
If any main branch fails, control goes to recovery.

Planner and Controller RecoveryNode
-----------------------------------

The planner branch now uses a ``Fallback`` with two reactive branches:

- ``ReactiveSequence`` ``CheckIfNewPathNeeded`` tries to keep using the current path.
- A second ``ReactiveSequence`` removes passed goals and computes a new path through remaining goals.

The ``Inverter`` wrapping ``GlobalUpdatedGoal`` is central in the first branch.
``GlobalUpdatedGoal`` returns ``SUCCESS`` when the global goal has changed in
the blackboard. The ``Inverter`` flips that result, so this condition returns
``SUCCESS`` when the goal has *not* changed.

Combined with ``IsGoalNearby`` and ``IsPathValid`` (after ``TruncatePathLocal``),
this allows the tree to continue following a valid current path and avoid
unnecessary cancel/replan cycles when no new goal is received.

If any check in ``CheckIfNewPathNeeded`` fails (for example, goal changed or
path invalid), the ``Fallback`` executes the second branch and replans.

In that branch, ``RemovePassedGoals`` culls passed waypoints from ``goals``
(radius ``0.7``), updates ``waypoint_statuses`` (``PENDING``, ``COMPLETED``,
``SKIPPED``, ``FAILED``), and ``ComputePathThroughPoses`` plans through the
remaining waypoints. Unlike ``NavigateToPose``, this planner uses a goal vector
(``goals``), not a single pose goal.

If planning fails, the planner ``RecoveryNode`` runs contextual planner
recovery (``WouldAPlannerRecoveryHelp`` then global costmap clearing).
Similarly, if ``FollowPath`` fails, controller contextual recovery clears the local
costmap.

System-Level Recovery Sequence - Auxiliary Branch
=================================================

If contextual planner/controller recoveries fail, the tree enters the
system-level recovery subtree.

This subtree is a ``ReactiveFallback`` with ``GoalUpdated`` first, so recovery
can be interrupted quickly when a new goal is preempted.

If the goal is unchanged, control moves to ``RoundRobin`` (``RecoveryActions``),
which rotates through actions in this order:

- local + global costmap clearing (``ClearingActions`` sequence)
- ``Spin``
- ``Wait``
- ``BackUp``

After each recovery attempt, the main navigation subtree is retried.
If it fails again, ``RoundRobin`` advances to the next recovery action.

Selector Nodes Note
===================

Although this tree uses default selections (``GridBased`` and ``FollowPath``),
the selector nodes (planner/controller/goal checker/progress checker/path
handler) support runtime algorithm switching via topics.

In many cases, subtree-level BT logic is still the preferred way to switch
behavior internally. However, selector nodes are useful when an external
application needs to control algorithm choice at runtime.

.. code-block:: xml

  <root BTCPP_format="4" main_tree_to_execute="NavigateThroughPosesWReplanningAndRecovery">
    <BehaviorTree ID="NavigateThroughPosesWReplanningAndRecovery">
      <RecoveryNode number_of_retries="6" name="NavigateRecovery">
        <PipelineSequence name="NavigateWithReplanning">
          <ProgressCheckerSelector selected_progress_checker="{selected_progress_checker}" default_progress_checker="progress_checker" topic_name="progress_checker_selector"/>
          <GoalCheckerSelector selected_goal_checker="{selected_goal_checker}" default_goal_checker="general_goal_checker" topic_name="goal_checker_selector"/>
          <PathHandlerSelector selected_path_handler="{selected_path_handler}" default_path_handler="PathHandler" topic_name="path_handler_selector"/>
          <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
          <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
          <RateController hz="0.333">
            <RecoveryNode number_of_retries="1" name="ComputePathThroughPoses">
              <Fallback name="FallbackComputePathToPose">
                <ReactiveSequence name="CheckIfNewPathNeeded">
                  <Inverter>
                    <GlobalUpdatedGoal/>
                  </Inverter>
                  <IsGoalNearby path="{path}" proximity_threshold="4.0" max_robot_pose_search_dist="1.5"/>
                  <TruncatePathLocal input_path="{path}" output_path="{remaining_path}" distance_forward="-1" distance_backward="0.0" />
                  <IsPathValid path="{remaining_path}"/>
                </ReactiveSequence>
                <ReactiveSequence>
                  <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7" input_waypoint_statuses="{waypoint_statuses}" output_waypoint_statuses="{waypoint_statuses}"/>
                  <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>
                </ReactiveSequence>
              </Fallback>
              <Sequence>
                <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
                <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
              </Sequence>
            </RecoveryNode>
          </RateController>
          <RecoveryNode number_of_retries="1" name="FollowPath">
            <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}" error_msg="{follow_path_error_msg}" goal_checker_id="{selected_goal_checker}" progress_checker_id="{selected_progress_checker}" path_handler_id="{selected_path_handler}" tracking_feedback="{tracking_feedback}"/>
            <Sequence>
              <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
              <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
            </Sequence>
          </RecoveryNode>
        </PipelineSequence>
        <Sequence>
          <Fallback>
            <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
            <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
          </Fallback>
          <ReactiveFallback name="RecoveryFallback">
            <GoalUpdated/>
            <RoundRobin name="RecoveryActions">
              <Sequence name="ClearingActions">
                <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
                <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
              </Sequence>
              <Spin name="SpinRecovery" spin_dist="1.57" error_code_id="{spin_error_code}" error_msg="{spin_error_msg}"/>
              <Wait name="WaitRecovery" wait_duration="5.0" error_code_id="{wait_error_code}" error_msg="{wait_error_msg}"/>
              <BackUp name="BackUpRecovery" backup_dist="0.30" backup_speed="0.15" error_code_id="{backup_error_code}" error_msg="{backup_error_msg}"/>
            </RoundRobin>
          </ReactiveFallback>
        </Sequence>
      </RecoveryNode>
    </BehaviorTree>
  </root>
