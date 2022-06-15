.. _behavior_trees:

Nav2 Behavior Trees
###################

.. toctree::
   :maxdepth: 1

   overview/nav2_specific_nodes.rst
   overview/detailed_behavior_tree_walkthrough.rst
   trees/nav_to_pose_recovery.rst
   trees/nav_through_poses_recovery.rst
   trees/nav_to_pose_and_pause_near_goal_obstacle.rst
   trees/nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.rst
   trees/follow_point.rst
   trees/odometry_calibration.rst

Nav2 is an incredibly reconfigurable project. It allows users to set many different plugin types, across behavior trees, core algorithms, status checkers, and more!
This section highlights some of the example behavior tree xml files provided by default in the project to do interesting tasks.
It should be noted that these can be modified for your specific application, or used as a guide to building your own application-specific behavior tree.
These are some exemplary examples of how you can reconfigure your navigation behavior significantly by using behavior trees.
Other behavior trees are provided by Nav2 in the ``nav2_bt_navigator`` package, but this section highlights the important ones.

A **very** basic, but functional, navigator can be seen below.

.. code-block:: xml

   <root main_tree_to_execute="MainTree">
     <BehaviorTree ID="MainTree">
       <PipelineSequence name="NavigateWithReplanning">
         <DistanceController distance="1.0">
           <ComputePathToPose goal="{goal}" path="{path}"/>
         </DistanceController>
         <FollowPath path="{path}"/>
       </PipelineSequence>
     </BehaviorTree>
   </root>

This behavior tree will simply plan a new path to ``goal`` every 1 meter (set by ``DistanceController``) using ``ComputePathToPose``.
If a new path is computed on the ``path`` blackboard variable, ``FollowPath`` will take this ``path`` and follow it using the server's default algorithm.

This tree contains:

- No recovery methods
- No retries on failure
- No selected planner or controller algorithms
- No nodes to contextually change settings for optimal performance
- No integration with automatic door, elevator, or other APIs
- No user provided custom BT nodes
- No subtrees for other behaviors like docking, following, etc.

All of this, and more, can be set and configured for your customized navigation logic in Nav2.
