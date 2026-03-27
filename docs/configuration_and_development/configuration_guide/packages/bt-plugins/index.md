# Behavior Tree XML Nodes { #behavior-tree-xml-nodes }

The [nav2_behavior_tree](https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree) package provides several navigation-specific nodes that are pre-registered and can be included in Behavior Trees.

Check this [introduction](https://www.behaviortree.dev/docs/learn-the-basics/BT_basics) to learn how behavior trees work and the difference between actions, conditions, controls and decorators.

Consider checking out the [Groot Tutorials][groot-tutorials] tutorial for using Groot to visualize and modify behavior trees.

## Action Plugins

<div class="grid cards" markdown>

- [Wait][wait]
- [Spin][spin]
- [BackUp][back-up]
- [DriveOnHeading][drive-on-heading]
- [AssistedTeleop][assisted-teleop]
- [ComputePathToPose][compute-path-to-pose]
- [ComputeRoute][compute-route]
- [ComputeAndTrackRoute][compute-and-track-route]
- [FollowPath][follow-path]
- [NavigateToPose][navigate-to-pose]
- [ClearEntireCostmap][clear-entire-costmap]
- [ClearCostmapExceptRegion][clear-costmap-except-region]
- [ClearCostmapAroundRobot][clear-costmap-around-robot]
- [ClearCostmapAroundPose][clear-costmap-around-pose]
- [ReinitializeGlobalLocalization][reinitialize-global-localization]
- [TruncatePath][truncate-path]
- [TruncatePathLocal][truncate-path-local]
- [PlannerSelector][planner-selector]
- [ControllerSelector][controller-selector]
- [SmootherSelector][smoother-selector]
- [GoalCheckerSelector][goal-checker-selector]
- [ProgressCheckerSelector][progress-checker-selector]
- [PathHandlerSelector][path-handler-selector]
- [NavigateThroughPoses][navigate-through-poses]
- [ComputePathThroughPoses][compute-path-through-poses]
- [ComputeCoveragePath][compute-coverage-path]
- [CancelCoverage][cancel-coverage]
- [RemovePassedGoals][remove-passed-goals]
- [RemoveInCollisionGoals][remove-in-collision-goals]
- [CancelControl][cancel-control]
- [CancelBackUp][cancel-back-up]
- [CancelSpin][cancel-spin]
- [CancelWait][cancel-wait]
- [CancelDriveOnHeading][cancel-drive-on-heading]
- [CancelAssistedTeleop][cancel-assisted-teleop]
- [CancelComputeAndTrackRoute][cancel-compute-and-track-route]
- [SmoothPath][smooth-path]
- [GetPoseFromPath][get-pose-from-path]
- [DockRobot][dock-robot]
- [UndockRobot][undock-robot]
- [ConcatenatePaths][concatenate-paths]
- [GetCurrentPose][get-current-pose]
- [AppendGoalPoseToGoals][append-goal-pose-to-goals]
- [ExtractRouteNodesAsGoals][extract-route-nodes-as-goals]
- [GetNextFewGoals][get-next-few-goals]
- [ToggleCollisionMonitor][toggle-collision-monitor]
- [FollowObject][follow-object]
- [CancelFollowObject][cancel-follow-object]

</div>

## Condition Plugins

<div class="grid cards" markdown>

- [GoalReached][goal-reached]
- [TransformAvailable][transform-available]
- [DistanceTraveled][distance-traveled]
- [GoalUpdated][goal-updated]
- [GlobalUpdatedGoal][global-updated-goal]
- [InitialPoseReceived][initial-pose-received]
- [IsGoalNearby][is-goal-nearby]
- [IsStuck][is-stuck]
- [IsStopped][is-stopped]
- [TimeExpired][time-expired]
- [IsBatteryLow][is-battery-low]
- [IsPathValid][is-path-valid]
- [IsPoseOccupied][is-pose-occupied]
- [IsWithinPathTrackingBounds][is-within-path-tracking-bounds]
- [PathExpiringTimer][path-expiring-timer]
- [AreErrorCodesPresent][are-error-codes-present]
- [WouldAControllerRecoveryHelp][would-a-controller-recovery-help]
- [WouldAPlannerRecoveryHelp][would-a-planner-recovery-help]
- [WouldASmootherRecoveryHelp][would-a-smoother-recovery-help]
- [WouldARouteRecoveryHelp][would-a-route-recovery-help]
- [IsBatteryCharging][is-battery-charging]
- [ArePosesNear][are-poses-near]

</div>

## Control Plugins

<div class="grid cards" markdown>

- [PipelineSequence][pipeline-sequence]
- [RoundRobin][round-robin]
- [RecoveryNode][recovery-node]
- [NonblockingSequence][nonblocking-sequence]
- [PersistentSequence][persistent-sequence]
- [PauseResumeController][pause-resume-controller]

</div>

## Decorator Plugins

<div class="grid cards" markdown>

- [RateController][rate-controller]
- [DistanceController][distance-controller]
- [SpeedController][speed-controller]
- [GoalUpdater][goal-updater]
- [PathLongerOnApproach][path-longer-on-approach]
- [SingleTrigger][single-trigger]
- [GoalUpdatedController][goal-updated-controller]

</div>

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
