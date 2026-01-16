.. _behavior_tree_navigate_on_route_graph_w_recovery:

Navigate on Route Graph with Recovery
#####################################

This behavior tree implements a different style of navigation than the other versions in this section.
Rather than using a freespace planner ``ComputePathToPose`` to plan a complete path to the goal, this behavior tree instead uses the Route Server to find a route to the goal through a pre-defined navigation graph.
This can be useful for navigating in large-scale environments where real-time planning in freespace for a long distance is not computationally feasible, where a map of the entire space is not possible to plan within, or where deterministic behavior and limited navigation zones/lanes/routes are demanded.

This tree computes a route through the environment using the ``ComputeRoute`` node which is executed on initialization and when either the goal is updated due to preemption (``GlobalUpdatedGoal``) or the current route path is invalid due to collision (``isPathValid``).
After which, if the robot's starting pose is too far from the first route node in the graph solution, it will use freespace planning to connect the robot's current pose to the first node in the route.
This is called the ``first mile`` and is computed using the ``ComputePathToPose`` node.
This may be removed if navigation only on the graph is required and you know that the robot will always be located on or near the graph.

The complimentary action occurs for ``last mile`` where the robot will use freespace planning to connect the last node in the route to the goal pose.
This is done using the ``ComputePathToPose`` node again and similarly can be removed if required.
The compute path, including the first and last mile paths are then smoothed in the smoother server to make the corners more natural and less sharp.
``FollowPath`` is then used to follow this path.

For a detailed description of the role of the selector nodes, recovery behaviors, or fallbacks, see the other behavior tree explanations in this section.

.. code-block:: xml

  <root BTCPP_format="4" main_tree_to_execute="NavigateOnRouteGraphWRecovery">
    <BehaviorTree ID="NavigateOnRouteGraphWRecovery">
      <RecoveryNode number_of_retries="6" name="NavigateRecovery">
        <PipelineSequence name="NavigateWithReplanning">
          <ControllerSelector selected_controller="{selected_controller}" default_controller="follow_path" topic_name="controller_selector"/>
          <PlannerSelector selected_planner="{selected_planner}" default_planner="grid_based" topic_name="planner_selector"/>
          <RecoveryNode number_of_retries="1" name="ComputeRoute">
            <RateController hz="0.5" name="ComputeRouteRateController">
                <Fallback>
                  <!-- Compute a new route if a new goal is found or the path is no longer valid -->
                  <ReactiveSequence>
                    <Inverter>
                      <GlobalUpdatedGoal/>
                    </Inverter>
                    <IsPathValid path="{path}"/> <!-- Base it on the complete connected 'path', not simply the 'route_path' -->
                  </ReactiveSequence>
                  <Sequence name="ComputeAndSmoothRoute">
                    <!-- Compute the route -->
                    <ComputeRoute goal="{goal}" path="{route_path}" route="{route}" use_poses="true" error_code_id="{compute_route_error_code}" error_msg="{compute_route_error_msg}"/>

                    <!-- Find if the route start node is far from the robot's current pose; if so, connect them for 'first mile'. -->
                    <ReactiveSequence>
                      <GetCurrentPose current_pose="{current_pose}"/>
                      <GetPoseFromPath path="{route_path}" index="0" pose="{route_start_pose}"/>
                      <Inverter>
                        <ArePosesNear ref_pose="{current_pose}" target_pose="{route_start_pose}" tolerance="0.3"/>
                      </Inverter>
                      <ComputePathToPose goal="{route_start_pose}" path="{first_mile_path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>
                      <ConcatenatePaths input_path1="{first_mile_path}" input_path2="{route_path}" output_path="{route_path}"/>
                    </ReactiveSequence>

                    <!-- Find if the route end node is far from the goal pose; if so, connect them for 'last mile'. -->
                    <ReactiveSequence>
                      <GetPoseFromPath path="{route_path}" index="-1" pose="{route_end_pose}"/>
                      <Inverter>
                        <ArePosesNear ref_pose="{goal}" target_pose="{route_end_pose}" tolerance="0.1"/>
                      </Inverter>
                      <ComputePathToPose start="{route_end_pose}" goal="{goal}" path="{last_mile_path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}" error_msg="{compute_path_error_msg}"/>
                      <ConcatenatePaths input_path1="{route_path}" input_path2="{last_mile_path}" output_path="{route_path}"/>
                    </ReactiveSequence>

                    <!-- Smooth the completed route -->
                    <SmoothPath unsmoothed_path="{route_path}" smoothed_path="{path}" smoother_id="route_smoother" error_code_id="{smoother_error_code}" error_msg="{smoother_error_msg}"/>
                  </Sequence>
                </Fallback>
              </RateController>
            <Sequence>
              <Fallback>
                <WouldARouteRecoveryHelp error_code="{compute_route_error_code}"/>
                <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
              </Fallback>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
          </RecoveryNode>
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
            <WouldAPlannerRecoveryHelp error_code="{compute_route_error_code}"/>
          </Fallback>
          <ReactiveFallback name="RecoveryFallback">
            <GoalUpdated/>
            <RoundRobin name="RecoveryActions">
              <Sequence name="ClearingActions">
                <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
                <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
              </Sequence>
              <Wait wait_duration="5.0" error_code_id="{wait_error_code}" error_msg="{wait_error_msg}"/>
              <BackUp backup_dist="0.30" backup_speed="0.15" error_code_id="{backup_error_code}" error_msg="{backup_error_msg}"/>
            </RoundRobin>
          </ReactiveFallback>
        </Sequence>
      </RecoveryNode>
    </BehaviorTree>
  </root>
