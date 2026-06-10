# Behavior Tree XML Nodes { #behavior-tree-xml-nodes }

The [nav2_behavior_tree](https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree) package provides several navigation-specific nodes that are pre-registered and can be included in Behavior Trees.

Check this [introduction](https://www.behaviortree.dev/docs/learn-the-basics/BT_basics) to learn how behavior trees work and the difference between actions, conditions, controls and decorators.

Consider checking out the [Groot Tutorials][groot-tutorials] tutorial for using Groot to visualize and modify behavior trees.

## Action Plugins

<div class="grid" markdown>

[Wait][wait]{ .md-button .md-button--primary }
[Spin][spin]{ .md-button .md-button--primary }
[BackUp][back-up]{ .md-button .md-button--primary }
[DriveOnHeading][drive-on-heading]{ .md-button .md-button--primary }
[AssistedTeleop][assisted-teleop]{ .md-button .md-button--primary }
[ComputePathToPose][compute-path-to-pose]{ .md-button .md-button--primary }
[ComputeRoute][compute-route]{ .md-button .md-button--primary }
[ComputeAndTrackRoute][compute-and-track-route]{ .md-button .md-button--primary }
[FollowPath][follow-path]{ .md-button .md-button--primary }
[NavigateToPose][navigate-to-pose]{ .md-button .md-button--primary }
[ClearEntireCostmap][clear-entire-costmap]{ .md-button .md-button--primary }
[ClearCostmapExceptRegion][clear-costmap-except-region]{ .md-button .md-button--primary }
[ClearCostmapAroundRobot][clear-costmap-around-robot]{ .md-button .md-button--primary }
[ClearCostmapAroundPose][clear-costmap-around-pose]{ .md-button .md-button--primary }
[ReinitializeGlobalLocalization][reinitialize-global-localization]{ .md-button .md-button--primary }
[TruncatePath][truncate-path]{ .md-button .md-button--primary }
[TruncatePathLocal][truncate-path-local]{ .md-button .md-button--primary }
[PlannerSelector][planner-selector]{ .md-button .md-button--primary }
[ControllerSelector][controller-selector]{ .md-button .md-button--primary }
[SmootherSelector][smoother-selector]{ .md-button .md-button--primary }
[GoalCheckerSelector][goal-checker-selector]{ .md-button .md-button--primary }
[ProgressCheckerSelector][progress-checker-selector]{ .md-button .md-button--primary }
[NavigateThroughPoses][navigate-through-poses]{ .md-button .md-button--primary }
[ComputePathThroughPoses][compute-path-through-poses]{ .md-button .md-button--primary }
<!-- [ComputeCoveragePath][compute-coverage-path]{ .md-button .md-button--primary } -->
<!-- [CancelCoverage][cancel-coverage]{ .md-button .md-button--primary } -->
[RemovePassedGoals][remove-passed-goals]{ .md-button .md-button--primary }
[CancelControl][cancel-control]{ .md-button .md-button--primary }
[CancelBackUp][cancel-back-up]{ .md-button .md-button--primary }
[CancelSpin][cancel-spin]{ .md-button .md-button--primary }
[CancelWait][cancel-wait]{ .md-button .md-button--primary }
[CancelDriveOnHeading][cancel-drive-on-heading]{ .md-button .md-button--primary }
[CancelAssistedTeleop][cancel-assisted-teleop]{ .md-button .md-button--primary }
[CancelComputeAndTrackRoute][cancel-compute-and-track-route]{ .md-button .md-button--primary }
[SmoothPath][smooth-path]{ .md-button .md-button--primary }
[GetPoseFromPath][get-pose-from-path]{ .md-button .md-button--primary }
[DockRobot][dock-robot]{ .md-button .md-button--primary }
[UndockRobot][undock-robot]{ .md-button .md-button--primary }
[ConcatenatePaths][concatenate-paths]{ .md-button .md-button--primary }
[GetCurrentPose][get-current-pose]{ .md-button .md-button--primary }
[ValidatePath][validate-path]{ .md-button .md-button--primary }

</div>

## Condition Plugins

<div class="grid" markdown>

[GoalReached][goal-reached]{ .md-button .md-button--primary }
[TransformAvailable][transform-available]{ .md-button .md-button--primary }
[DistanceTraveled][distance-traveled]{ .md-button .md-button--primary }
[GoalUpdated][goal-updated]{ .md-button .md-button--primary }
[GlobalUpdatedGoal][global-updated-goal]{ .md-button .md-button--primary }
[InitialPoseReceived][initial-pose-received]{ .md-button .md-button--primary }
[IsStuck][is-stuck]{ .md-button .md-button--primary }
[TimeExpired][time-expired]{ .md-button .md-button--primary }
[IsBatteryLow][is-battery-low]{ .md-button .md-button--primary }
[PathExpiringTimer][path-expiring-timer]{ .md-button .md-button--primary }
[AreErrorCodesPresent][are-error-codes-present]{ .md-button .md-button--primary }
[WouldAControllerRecoveryHelp][would-a-controller-recovery-help]{ .md-button .md-button--primary }
[WouldAPlannerRecoveryHelp][would-a-planner-recovery-help]{ .md-button .md-button--primary }
[WouldASmootherRecoveryHelp][would-a-smoother-recovery-help]{ .md-button .md-button--primary }
[IsBatteryCharging][is-battery-charging]{ .md-button .md-button--primary }
[ArePosesNear][are-poses-near]{ .md-button .md-button--primary }

</div>

## Control Plugins

<div class="grid" markdown>

[PipelineSequence][pipeline-sequence]{ .md-button .md-button--primary }
[RoundRobin][round-robin]{ .md-button .md-button--primary }
[RecoveryNode][recovery-node]{ .md-button .md-button--primary }

</div>

## Decorator Plugins

<div class="grid" markdown>

[RateController][rate-controller]{ .md-button .md-button--primary }
[DistanceController][distance-controller]{ .md-button .md-button--primary }
[SpeedController][speed-controller]{ .md-button .md-button--primary }
[GoalUpdater][goal-updater]{ .md-button .md-button--primary }
[PathLongerOnApproach][path-longer-on-approach]{ .md-button .md-button--primary }
[SingleTrigger][single-trigger]{ .md-button .md-button--primary }
[GoalUpdatedController][goal-updated-controller]{ .md-button .md-button--primary }

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
