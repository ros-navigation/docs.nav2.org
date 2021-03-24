.. _configuring_behavior_tree_xml:

Behavior Tree XML Nodes
#################################

The nav2_behavior_tree_ package provides several navigation-specific nodes that are pre-registered and can be included in Behavior Trees.

.. _nav2_behavior_tree: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree

Check this introduction_ to learn how behavior trees work and the difference between actions, conditions, controls and decorators.

.. _introduction: https://www.behaviortree.dev/bt_basics/

Note: ``SingleTrigger`` BT Node is not listed below because it contains no parameters.

Action Plugins
**************

.. toctree::
  :maxdepth: 1

  bt-plugins/actions/Wait.rst
  bt-plugins/actions/Spin.rst
  bt-plugins/actions/BackUp.rst
  bt-plugins/actions/ComputePathToPose.rst
  bt-plugins/actions/FollowPath.rst
  bt-plugins/actions/NavigateToPose.rst
  bt-plugins/actions/ClearEntireCostmap.rst
  bt-plugins/actions/ClearCostmapExceptRegion.rst
  bt-plugins/actions/ClearCostmapAroundRobot.rst
  bt-plugins/actions/ReinitializeGlobalLocalization.rst
  bt-plugins/actions/TruncatePath.rst
  bt-plugins/actions/PlannerSelector.rst
  bt-plugins/actions/ControllerSelector.rst
  bt-plugins/actions/GoalCheckerSelector.rst


Condition Plugins
*****************

.. toctree::
  :maxdepth: 1

  bt-plugins/conditions/GoalReached.rst
  bt-plugins/conditions/TransformAvailable.rst
  bt-plugins/conditions/DistanceTraveled.rst
  bt-plugins/conditions/GoalUpdated.rst
  bt-plugins/conditions/InitialPoseReceived.rst
  bt-plugins/conditions/IsStuck.rst
  bt-plugins/conditions/TimeExpired.rst
  bt-plugins/conditions/IsBatteryLow.rst

Control Plugins
***************

.. toctree::
  :maxdepth: 1

  bt-plugins/controls/PipelineSequence.rst
  bt-plugins/controls/RoundRobin.rst
  bt-plugins/controls/RecoveryNode.rst

Decorator Plugins
*****************

.. toctree::
  :maxdepth: 1

  bt-plugins/decorators/RateController.rst
  bt-plugins/decorators/DistanceController.rst
  bt-plugins/decorators/SpeedController.rst
  bt-plugins/decorators/GoalUpdater.rst


Example
*******

This Behavior Tree replans the global path periodically at 1 Hz and it also has
recovery actions.

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
