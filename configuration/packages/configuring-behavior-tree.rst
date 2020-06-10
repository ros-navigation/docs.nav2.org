.. _configuring_behavior_tree:

Configuring the behavior tree XML
#################################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_behavior_tree

Behavior Tree Plugins
*********************

Actions
=======

  .. toctree::
    :maxdepth: 1

    bt-plugins/actions/Wait.rst
    bt-plugins/actions/Spin.rst
    bt-plugins/actions/BackUp.rst
    bt-plugins/actions/ComputePathToPose.rst
    bt-plugins/actions/FollowPath.rst
    bt-plugins/actions/NavigateToPose.rst
    bt-plugins/actions/ClearEntireCostmap.rst
    bt-plugins/actions/ReinitializeGlobalLocalization.rst


Conditions
==========

  .. toctree::
    :maxdepth: 1

    bt-plugins/conditions/GoalReached.rst
    bt-plugins/conditions/TransformAvailable.rst

Controls
========

  .. toctree::
    :maxdepth: 1

    bt-plugins/controls/RecoveryNode.rst

Decorators
============

  .. toctree::
    :maxdepth: 1

    bt-plugins/decorators/RateController.rst
    bt-plugins/decorators/DistanceController.rst

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
