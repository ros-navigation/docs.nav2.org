.. _behavior_tree_nav_to_pose_and_replan_if_path_invalid:

Navigate To Pose With Consistent Replanning And If Path Becomes Invalid
#######################################################################

This behavior tree implements a significantly more mature version of the behavior tree on :ref:`behavior_trees`.
It navigates from a starting point to a single point goal in freespace.
It contains both use of custom recoveries in specific sub-contexts as well as a global recovery subtree for system-level failures.
It also provides the opportunity for users to retry tasks multiple times before returning a failed state.

The ``ComputePathToPose`` and ``FollowPath`` BT nodes both also specify their algorithms to utilize.
By convention we name these by the style of algorithms that they are (e.g. not ``DWB`` but rather ``FollowPath``) such that a behavior tree or application developer need not worry about the technical specifics. They just want to use a path following controller.

In this behavior tree, we attempt to retry the entire navigation task 6 times before returning to the caller that the task has failed.
This allows the navigation system ample opportunity to try to recovery from failure conditions or wait for transient issues to pass, such as crowding from people or a temporary sensor failure.

In nominal execution, replanning can be triggered by an a invalid previous path, a new goal or if a new path has not been created for 10 seconds.
If the planner or controller fails, it will trigger contextually aware recoveries in its subtree.
Currently, the recoveries will clear the global costmap if the planner fails and clear the local costmap if the controller fails.
Additional context-specific recoveries can be added to these subtrees.

If these contextual recoveries fail, this behavior tree enters the recovery subtree.
This subtree is reserved for system-level failures to help resolve issues like the robot being stuck or in a bad spot.
This subtree has the ``GoalUpdated`` BT node which ticks every iteration to ensure responsiveness of new goals.
Next, the recovery subtree will attempt the following recoveries: costmap clearing operations, spinning, waiting, and backing up.
After each of the recoveries in the subtree, the main navigation subtree will be reattempted.
If it continues to fail, the next recovery in the recovery subtree is ticked.

While this behavior tree does not make use of it, the ``PlannerSelector``, ``ControllerSelector``, and ``GoalCheckerSelector`` behavior tree nodes can also be helpful. Rather than hardcoding the algorithm to use (``GridBased`` and ``FollowPath``), these behavior tree nodes will allow a user to dynamically change the algorithm used in the navigation system via a ROS topic. It may be instead advisable to create different subtree contexts using condition nodes with specified algorithms in their most useful and unique situations. However, the selector nodes can be a useful way to change algorithms from an external application rather than via internal behavior tree control flow logic. It is better to implement changes through behavior tree methods, but we understand that many professional users have external applications to dynamically change settings of their navigators.

.. code-block:: xml

  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <RecoveryNode number_of_retries="6" name="NavigateRecovery">
        <PipelineSequence>
          <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
          <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
          <RateController hz="1.0" name="RateControllerComputePathToPose">
            <RecoveryNode number_of_retries="1" name="RecoveryComputePathToPose">
              <Fallback name="FallbackComputePathToPose">
                <ReactiveSequence name="CheckIfNewPathNeeded">
                  <Inverter>
                    <GlobalUpdatedGoal/>
                  </Inverter>
                  <IsPathValid path="{path}"/>
                </ReactiveSequence>
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>
              </Fallback>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </RecoveryNode>
          </RateController>
          <RecoveryNode number_of_retries="1" name="RecoveryFollowPath">
            <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}" error_msg="{follow_path_error_msg}"/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
          </RecoveryNode>
        </PipelineSequence>
        <ReactiveFallback name="FallbackRecoveries">
          <GoalUpdated/>
          <RoundRobin name="RecoveryActions">
            <Sequence name="ClearingActions">
              <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
            <Spin name="SpinRecovery" spin_dist="1.57" error_code_id="{spin_error_code}" error_msg="{spin_error_msg}"/>
            <Wait name="WaitRecovery" wait_duration="5.0"/>
            <BackUp name="BackUpRecovery" backup_dist="0.30" backup_speed="0.05" error_code_id="{backup_error_code}" error_msg="{backup_error_msg}"/>
          </RoundRobin>
        </ReactiveFallback>
      </RecoveryNode>
    </BehaviorTree>
  </root>
