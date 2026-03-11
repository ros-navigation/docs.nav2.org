# Behavior Tree XML Nodes

The [nav2_behavior_tree](https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree) package provides several navigation-specific nodes that are pre-registered and can be included in Behavior Trees.

Check this [introduction](https://www.behaviortree.dev/docs/learn-the-basics/BT_basics) to learn how behavior trees work and the difference between actions, conditions, controls and decorators.

Consider checking out the [Groot Tutorials](../../tutorials/docs/using_groot.md#groot-tutorials) tutorial for using Groot to visualize and modify behavior trees.

## Action Plugins

* [Wait](bt-plugins/actions/Wait.md)
* [Spin](bt-plugins/actions/Spin.md)
* [BackUp](bt-plugins/actions/BackUp.md)
* [DriveOnHeading](bt-plugins/actions/DriveOnHeading.md)
* [AssistedTeleop](bt-plugins/actions/AssistedTeleop.md)
* [ComputePathToPose](bt-plugins/actions/ComputePathToPose.md)
* [ComputeRoute](bt-plugins/actions/ComputeRoute.md)
* [ComputeAndTrackRoute](bt-plugins/actions/ComputeAndTrackRoute.md)
* [FollowPath](bt-plugins/actions/FollowPath.md)
* [NavigateToPose](bt-plugins/actions/NavigateToPose.md)
* [ClearEntireCostmap](bt-plugins/actions/ClearEntireCostmap.md)
* [ClearCostmapExceptRegion](bt-plugins/actions/ClearCostmapExceptRegion.md)
* [ClearCostmapAroundRobot](bt-plugins/actions/ClearCostmapAroundRobot.md)
* [ClearCostmapAroundPose](bt-plugins/actions/ClearCostmapAroundPose.md)
* [ReinitializeGlobalLocalization](bt-plugins/actions/ReinitializeGlobalLocalization.md)
* [TruncatePath](bt-plugins/actions/TruncatePath.md)
* [TruncatePathLocal](bt-plugins/actions/TruncatePathLocal.md)
* [PlannerSelector](bt-plugins/actions/PlannerSelector.md)
* [ControllerSelector](bt-plugins/actions/ControllerSelector.md)
* [SmootherSelector](bt-plugins/actions/SmootherSelector.md)
* [GoalCheckerSelector](bt-plugins/actions/GoalCheckerSelector.md)
* [ProgressCheckerSelector](bt-plugins/actions/ProgressCheckerSelector.md)
* [PathHandlerSelector](bt-plugins/actions/PathHandlerSelector.md)
* [NavigateThroughPoses](bt-plugins/actions/NavigateThroughPoses.md)
* [ComputePathThroughPoses](bt-plugins/actions/ComputePathThroughPoses.md)
* [ComputeCoveragePath](bt-plugins/actions/ComputeCoveragePath.md)
* [CancelCoverage](bt-plugins/actions/CancelCoverage.md)
* [RemovePassedGoals](bt-plugins/actions/RemovePassedGoals.md)
* [RemoveInCollisionGoals](bt-plugins/actions/RemoveInCollisionGoals.md)
* [CancelControl](bt-plugins/actions/CancelControl.md)
* [CancelBackUp](bt-plugins/actions/CancelBackUp.md)
* [CancelSpin](bt-plugins/actions/CancelSpin.md)
* [CancelWait](bt-plugins/actions/CancelWait.md)
* [CancelDriveOnHeading](bt-plugins/actions/CancelDriveOnHeading.md)
* [CancelAssistedTeleop](bt-plugins/actions/CancelAssistedTeleop.md)
* [CancelComputeAndTrackRoute](bt-plugins/actions/CancelComputeAndTrackRoute.md)
* [SmoothPath](bt-plugins/actions/Smooth.md)
* [GetPoseFromPath](bt-plugins/actions/GetPoseFromPath.md)
* [DockRobot](bt-plugins/actions/DockRobot.md)
* [UndockRobot](bt-plugins/actions/UndockRobot.md)
* [ConcatenatePaths](bt-plugins/actions/ConcatenatePaths.md)
* [GetCurrentPose](bt-plugins/actions/GetCurrentPose.md)
* [AppendGoalPoseToGoals](bt-plugins/actions/AppendGoalPoseToGoals.md)
* [ExtractRouteNodesAsGoals](bt-plugins/actions/ExtractRouteNodesAsGoals.md)
* [GetNextFewGoals](bt-plugins/actions/GetNextFewGoals.md)
* [ToggleCollisionMonitor](bt-plugins/actions/ToggleCollisionMonitor.md)
* [FollowObject](bt-plugins/actions/FollowObject.md)
* [CancelFollowObject](bt-plugins/actions/CancelFollowObject.md)

## Condition Plugins

* [GoalReached](bt-plugins/conditions/GoalReached.md)
* [TransformAvailable](bt-plugins/conditions/TransformAvailable.md)
* [DistanceTraveled](bt-plugins/conditions/DistanceTraveled.md)
* [GoalUpdated](bt-plugins/conditions/GoalUpdated.md)
* [GlobalUpdatedGoal](bt-plugins/conditions/GlobalUpdatedGoal.md)
* [InitialPoseReceived](bt-plugins/conditions/InitialPoseReceived.md)
* [IsGoalNearby](bt-plugins/conditions/IsGoalNearby.md)
* [IsStuck](bt-plugins/conditions/IsStuck.md)
* [IsStopped](bt-plugins/conditions/IsStopped.md)
* [TimeExpired](bt-plugins/conditions/TimeExpired.md)
* [IsBatteryLow](bt-plugins/conditions/IsBatteryLow.md)
* [IsPathValid](bt-plugins/conditions/IsPathValid.md)
* [IsPoseOccupied](bt-plugins/conditions/IsPoseOccupied.md)
* [IsWithinPathTrackingBounds](bt-plugins/conditions/IsWithinPathTrackingBounds.md)
* [PathExpiringTimer](bt-plugins/conditions/PathExpiringTimer.md)
* [AreErrorCodesPresent](bt-plugins/conditions/AreErrorCodesPresent.md)
* [WouldAControllerRecoveryHelp](bt-plugins/conditions/WouldAControllerRecoveryHelp.md)
* [WouldAPlannerRecoveryHelp](bt-plugins/conditions/WouldAPlannerRecoveryHelp.md)
* [WouldASmootherRecoveryHelp](bt-plugins/conditions/WouldASmootherRecoveryHelp.md)
* [WouldARouteRecoveryHelp](bt-plugins/conditions/WouldARouteRecoveryHelp.md)
* [IsBatteryCharging](bt-plugins/conditions/IsBatteryCharging.md)
* [ArePosesNear](bt-plugins/conditions/ArePosesNear.md)

## Control Plugins

* [PipelineSequence](bt-plugins/controls/PipelineSequence.md)
* [RoundRobin](bt-plugins/controls/RoundRobin.md)
* [RecoveryNode](bt-plugins/controls/RecoveryNode.md)
* [NonblockingSequence](bt-plugins/controls/NonblockingSequence.md)
* [PersistentSequence](bt-plugins/controls/PersistentSequence.md)
* [PauseResumeController](bt-plugins/controls/PauseResumeController.md)

## Decorator Plugins

* [RateController](bt-plugins/decorators/RateController.md)
* [DistanceController](bt-plugins/decorators/DistanceController.md)
* [SpeedController](bt-plugins/decorators/SpeedController.md)
* [GoalUpdater](bt-plugins/decorators/GoalUpdater.md)
* [PathLongerOnApproach](bt-plugins/decorators/PathLongerOnApproach.md)
* [SingleTrigger](bt-plugins/decorators/SingleTrigger.md)
* [GoalUpdatedController](bt-plugins/decorators/GoalUpdatedController.md)

## Example

This Behavior Tree replans the global path periodically at 1 Hz and it also has
recovery actions.

```xml
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
```
