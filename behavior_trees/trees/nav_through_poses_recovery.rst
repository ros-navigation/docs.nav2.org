.. _behavior_tree_nav_through_poses:

Navigate Through Poses
######################

This behavior tree implements a navigation behavior from a starting point, through many intermediary hard pose constraints, to a final goal in freespace.
It contains both use of custom behaviors for recovery in specific sub-contexts as well as a global recovery subtree for system-level failures.
It also provides the opportunity for users to retry tasks multiple times before returning a failed state.

The ``ComputePathThroughPoses`` and ``FollowPath`` BT nodes both also specify their algorithms to utilize.
By convention we name these by the style of algorithms that they are (e.g. not ``DWB`` but rather ``FollowPath``) such that a behavior tree or application developer need not worry about the technical specifics. They just want to use a path following controller.

In this behavior tree, we attempt to retry the entire navigation task 6 times before returning to the caller that the task has failed.
This allows the navigation system ample opportunity to try to recovery from failure conditions or wait for transient issues to pass, such as crowding from people or a temporary sensor failure.

In nominal execution, this will replan the path at every 3 seconds and pass that path onto the controller, similar to the behavior tree in :ref:`behavior_trees`.
The planner though is now ``ComputePathThroughPoses`` taking a vector, ``goals``, rather than a single pose ``goal`` to plan to.
The ``RemovePassedGoals`` node is used to cull out ``goals`` that the robot has passed on its path.
In this case, it is set to remove a pose from the poses when the robot is within ``0.5`` of the goal and it is the next goal in the list.
This is implemented such that replanning can be computed after the robot has passed by some of the intermediary poses and not continue to try to replan through them in the future.
This time, if the planner fails, it will trigger contextually aware recoveries in its subtree, clearing the global costmap.
Additional recoveries can be added here for additional context-specific recoveries, such as trying another algorithm.

Similarly, the controller has similar logic. If it fails, it also attempts a costmap clearing of the local costmap impacting the controller.
It is worth noting the ``GoalUpdated`` node in the reactive fallback.
This allows us to exit recovery conditions when a new goal has been passed to the navigation system through a preemption.
This ensures that the navigation system will be very responsive immediately when a new goal is issued, even when the last goal was in an attempted recovery.

If these contextual recoveries fail, this behavior tree enters the recovery subtree.
This subtree is reserved for system-level failures to help resolve issues like the robot being stuck or in a bad spot.
This subtree also has the ``GoalUpdated`` BT node it ticks every iteration to ensure responsiveness of new goals.
Next, the recovery subtree will tick the costmap clearing operations, spinning, waiting, and backing up.
After each of the recoveries in the subtree, the main navigation subtree will be reattempted.
If it continues to fail, the next recovery in the recovery subtree is ticked.

While this behavior tree does not make use of it, the ``PlannerSelector``, ``ControllerSelector``, and ``GoalCheckerSelector`` behavior tree nodes can also be helpful. Rather than hardcoding the algorithm to use (``GridBased`` and ``FollowPath``), these behavior tree nodes will allow a user to dynamically change the algorithm used in the navigation system via a ROS topic. It may be instead advisable to create different subtree contexts using condition nodes with specified algorithms in their most useful and unique situations. However, the selector nodes can be a useful way to change algorithms from an external application rather than via internal behavior tree control flow logic. It is better to implement changes through behavior tree methods, but we understand that many professional users have external applications to dynamically change settings of their navigators.

.. code-block:: xml

  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <RecoveryNode number_of_retries="6" name="NavigateRecovery">
        <PipelineSequence name="NavigateWithReplanning">
          <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
          <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
          <RateController hz="0.333">
            <RecoveryNode number_of_retries="1" name="ComputePathThroughPoses">
              <ReactiveSequence>
                <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"/>
                <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>
              </ReactiveSequence>
              <Sequence>
                <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
                <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
              </Sequence>
            </RecoveryNode>
          </RateController>
          <RecoveryNode number_of_retries="1" name="FollowPath">
            <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}" error_msg="{follow_path_error_msg}"/>
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
              <Spin spin_dist="1.57" error_code_id="{spin_error_code} error_msg="{spin_error_msg}"/>
              <Wait wait_duration="5.0"/>
              <BackUp backup_dist="0.30" backup_speed="0.05" error_code_id="{backup_error_code}" error_msg="{backup_error_msg}"/>
            </RoundRobin>
          </ReactiveFallback>
        </Sequence>
      </RecoveryNode>
    </BehaviorTree>
  </root>
