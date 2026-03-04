.. _behavior_tree_nav_to_pose:

Navigate To Pose
################

This behavior tree implements a significantly more mature version of the behavior tree on :ref:`behavior_trees`.
It navigates from a starting point to a single point goal in freespace.
It contains both use of custom recovery behaviors in specific sub-contexts as well as a global recovery subtree for system-level failures.
It also provides the opportunity for users to retry tasks multiple times before returning a failed state.

The ``ComputePathToPose`` and ``FollowPath`` BT nodes both also specify their algorithms to utilize.
By convention we name these by the style of algorithms that they are (e.g. not ``DWB`` but rather ``FollowPath``) such that a behavior tree or application developer need not worry about the technical specifics. They just want to use a path following controller.

In this behavior tree, we attempt to retry the entire navigation task 6 times before returning to the caller that the task has failed.
This allows the navigation system ample opportunity to try to recovery from failure conditions or wait for transient issues to pass, such as crowding from people or a temporary sensor failure.

In nominal execution, this will replan the path at every second if not close enough to goal and pass that path onto the controller, similar to the behavior tree in :ref:`behavior_trees`.
However, this time, if the planner fails, it will trigger contextually aware recovery behaviors in its subtree, clearing the global costmap.
Additional recovery behaviors can be added here for additional context-specific recoveries, such as trying another algorithm.

Similarly, the controller has similar logic. If it fails, it also attempts a costmap clearing of the local costmap impacting the controller.
It is worth noting the ``GoalUpdated`` node in the reactive fallback.
This allows us to exit recovery conditions when a new goal has been passed to the navigation system through a preemption.
This ensures that the navigation system will be very responsive immediately when a new goal is issued, even when the last goal was in an attempted recovery.

If these contextual recoveries fail, this behavior tree enters the recovery subtree.
This subtree is reserved for system-level failures to help resolve issues like the robot being stuck or in a bad spot.
This subtree also has the ``GoalUpdated`` BT node it ticks every iteration to ensure responsiveness of new goals.
Next, the recovery subtree will the recoveries: costmap clearing operations, spinning, waiting, and backing up.
After each of the recoveries in the subtree, the main navigation subtree will be reattempted.
If it continues to fail, the next recovery in the recovery subtree is ticked.

While this behavior tree does not make use of it, the ``PlannerSelector``, ``ControllerSelector``, ``GoalCheckerSelector``, ``ProgressCheckerSelector``, and ``PathHandlerSelector`` behavior tree nodes can also be helpful. Rather than hardcoding the algorithm to use (``GridBased`` and ``FollowPath``), these behavior tree nodes will allow a user to dynamically change the algorithm used in the navigation system via a ROS topic. It may be instead advisable to create different subtree contexts using condition nodes with specified algorithms in their most useful and unique situations. However, the selector nodes can be a useful way to change algorithms from an external application rather than via internal behavior tree control flow logic. It is better to implement changes through behavior tree methods, but we understand that many professional users have external applications to dynamically change settings of their navigators.

.. code-block:: xml

  <root BTCPP_format="4" main_tree_to_execute="NavigateToPoseWReplanningAndRecovery">
    <BehaviorTree ID="NavigateToPoseWReplanningAndRecovery">
      <RecoveryNode number_of_retries="6" name="NavigateRecovery">
        <PipelineSequence name="NavigateWithReplanning">
          <ProgressCheckerSelector selected_progress_checker="{selected_progress_checker}" default_progress_checker="progress_checker" topic_name="progress_checker_selector"/>
          <GoalCheckerSelector selected_goal_checker="{selected_goal_checker}" default_goal_checker="general_goal_checker" topic_name="goal_checker_selector"/>
          <PathHandlerSelector selected_path_handler="{selected_path_handler}" default_path_handler="PathHandler" topic_name="path_handler_selector"/>
          <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
          <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
          <RateController hz="1.0">
            <RecoveryNode number_of_retries="1" name="ComputePathToPose">
              <Fallback name="FallbackComputePathToPose">
                <ReactiveSequence name="CheckIfNewPathNeeded">
                  <Inverter>
                    <GlobalUpdatedGoal/>
                  </Inverter>
                  <IsGoalNearby path="{path}" proximity_threshold="4.0" max_robot_pose_search_dist="1.5"/>
                  <TruncatePathLocal input_path="{path}" output_path="{remaining_path}" distance_forward="-1" distance_backward="0.0" />
                  <IsPathValid path="{remaining_path}"/>
                </ReactiveSequence>
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>
              </Fallback>
              <Sequence>
                <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
                <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
              </Sequence>
            </RecoveryNode>
          </RateController>
          <RecoveryNode number_of_retries="1" name="FollowPath">
            <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}" error_msg="{follow_path_error_msg}" tracking_feedback="{tracking_feedback}"/>
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
              <Spin spin_dist="1.57" error_code_id="{spin_error_code}" error_msg="{spin_error_msg}"/>
              <Wait wait_duration="5.0" error_code_id="{wait_error_code}" error_msg="{wait_error_msg}"/>
              <BackUp backup_dist="0.30" backup_speed="0.15" error_code_id="{backup_error_code}" error_msg="{backup_error_msg}"/>
            </RoundRobin>
          </ReactiveFallback>
        </Sequence>
      </RecoveryNode>
    </BehaviorTree>
  </root>
