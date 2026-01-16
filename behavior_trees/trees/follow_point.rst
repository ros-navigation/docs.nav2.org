.. _behavior_tree_follow_point:

Follow Dynamic Point
####################


This behavior tree implements a navigation behavior from a starting point, attempting to follow a dynamic point over time.
This "dynamic point" could be a person, another robot, a virtual carrot, anything.
The only requirement is that the pose you'd like to follow is published to the topic outlined in the ``GoalUpdater`` BT node.

In this tree, we replan at 1 hz just as we did in :ref:`behavior_tree_nav_to_pose` using the ``ComputePathToPose`` node.
However, this time when we replan, we update the ``goal`` based on the newest information in on the updated goal topic.
After we plan a path to this dynamic point, we use the ``TruncatePath`` node to remove path points from the end of the path near the dynamic point.
This behavior tree node is useful so that the robot always remains at least ``distance`` away from the obstacle, even if it stops.
It also smooths out any off path behavior involved with trying to path plan towards a probably occupied space in the costmap.

After the new path to the dynamic point is computed and truncated, it is again passed to the controller via the ``FollowPath`` node.
However, note that it is under a ``KeepRunningUntilFailure`` decorator node ensuring the controller continues to execute until a failure mode.
This behavior tree will execute infinitely in time until the navigation request is preempted or cancelled.

.. code-block:: xml

  <root BTCPP_format="4" main_tree_to_execute="FollowPoint">
    <BehaviorTree ID="FollowPoint">
      <PipelineSequence name="NavigateWithReplanning">
        <ControllerSelector selected_controller="{selected_controller}" default_controller="follow_path" topic_name="controller_selector"/>
        <PlannerSelector selected_planner="{selected_planner}" default_planner="grid_based" topic_name="planner_selector"/>
        <RateController hz="1.0">
          <Sequence>
            <GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
              <ComputePathToPose goal="{updated_goal}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>
            </GoalUpdater>
          <TruncatePath distance="1.0" input_path="{path}" output_path="{truncated_path}"/>
          </Sequence>
        </RateController>
        <KeepRunningUntilFailure>
          <FollowPath path="{truncated_path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}" error_msg="{follow_path_error_msg}"/>
        </KeepRunningUntilFailure>
      </PipelineSequence>
    </BehaviorTree>
  </root>
