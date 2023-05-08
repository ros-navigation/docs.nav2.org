.. _configuring_behavior_tree_xml:

Behavior Tree XML Nodes
#######################

The nav2_behavior_tree_ package provides several navigation-specific nodes that are pre-registered and can be included in Behavior Trees.

.. _nav2_behavior_tree: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree

Check this introduction_ to learn how behavior trees work and the difference between actions, conditions, controls and decorators.

.. _introduction: https://www.behaviortree.dev/bt_basics/

Consider checking out the :ref:`groot_introduction` tutorial for using Groot to visualize and modify behavior trees.

Action Plugins
**************

.. toctree::
  :maxdepth: 1

  bt-plugins/actions/Wait.rst
  bt-plugins/actions/Spin.rst
  bt-plugins/actions/BackUp.rst
  bt-plugins/actions/DriveOnHeading.rst
  bt-plugins/actions/AssistedTeleop.rst
  bt-plugins/actions/ComputePathToPose.rst
  bt-plugins/actions/FollowPath.rst
  bt-plugins/actions/NavigateToPose.rst
  bt-plugins/actions/ClearEntireCostmap.rst
  bt-plugins/actions/ClearCostmapExceptRegion.rst
  bt-plugins/actions/ClearCostmapAroundRobot.rst
  bt-plugins/actions/ReinitializeGlobalLocalization.rst
  bt-plugins/actions/TruncatePath.rst
  bt-plugins/actions/TruncatePathLocal.rst
  bt-plugins/actions/PlannerSelector.rst
  bt-plugins/actions/ControllerSelector.rst
  bt-plugins/actions/SmootherSelector.rst
  bt-plugins/actions/GoalCheckerSelector.rst
  bt-plugins/actions/NavigateThroughPoses.rst
  bt-plugins/actions/ComputePathThroughPoses.rst
  bt-plugins/actions/RemovePassedGoals.rst
  bt-plugins/actions/CancelControl.rst
  bt-plugins/actions/CancelBackUp.rst
  bt-plugins/actions/CancelSpin.rst
  bt-plugins/actions/CancelWait.rst
  bt-plugins/actions/CancelDriveOnHeading.rst
  bt-plugins/actions/CancelAssistedTeleop.rst
  bt-plugins/actions/Smooth.rst

Condition Plugins
*****************

.. toctree::
  :maxdepth: 1

  bt-plugins/conditions/GoalReached.rst
  bt-plugins/conditions/TransformAvailable.rst
  bt-plugins/conditions/DistanceTraveled.rst
  bt-plugins/conditions/GoalUpdated.rst
  bt-plugins/conditions/GloballyUpdatedGoal.rst
  bt-plugins/conditions/InitialPoseReceived.rst
  bt-plugins/conditions/IsStuck.rst
  bt-plugins/conditions/TimeExpired.rst
  bt-plugins/conditions/IsBatteryLow.rst
  bt-plugins/conditions/IsPathValid.rst
  bt-plugins/conditions/PathExpiringTimer.rst
  bt-plugins/conditions/AreErrorCodesPresent.rst
  bt-plugins/conditions/WouldAControllerRecoveryHelp.rst
  bt-plugins/conditions/WouldAPlannerRecoveryHelp.rst
  bt-plugins/conditions/WouldASmootherRecoveryHelp.rst
  bt-plugins/conditions/IsBatteryCharging.rst

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
  bt-plugins/decorators/PathLongerOnApproach.rst
  bt-plugins/decorators/SingleTrigger.rst

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
