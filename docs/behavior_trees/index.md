<a id="behavior-trees"></a>

# Nav2 Behavior Trees

* [Introduction To Nav2 Specific Nodes](overview/nav2_specific_nodes.md)
* [Detailed Behavior Tree Walkthrough](overview/detailed_behavior_tree_walkthrough.md)
* [Navigate To Pose](trees/nav_to_pose_recovery.md)
* [Navigate Through Poses](trees/nav_through_poses_recovery.md)
* [Navigate To Pose and Pause Near Goal-Obstacle](trees/nav_to_pose_and_pause_near_goal_obstacle.md)
* [Navigate To Pose With Consistent Replanning And If Path Becomes Invalid](trees/nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.md)
* [Navigate on Route Graph with Recovery](trees/navigate_on_route_graph_w_recovery.md)
* [Follow Dynamic Point](trees/follow_point.md)
* [Odometry Calibration](trees/odometry_calibration.md)

Nav2 is an incredibly reconfigurable project. It allows users to set many different plugin types, across behavior trees, core algorithms, status checkers, and more!
This section highlights some of the example behavior tree xml files provided by default in the project to do interesting tasks.
It should be noted that these can be modified for your specific application, or used as a guide to building your own application-specific behavior tree.
These are some exemplary examples of how you can reconfigure your navigation behavior significantly by using behavior trees.
Other behavior trees are provided by Nav2 in the `nav2_bt_navigator` package, but this section highlights the important ones.

A **very** basic, but functional, navigator can be seen below.

```xml
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
```

This behavior tree will simply plan a new path to `goal` every 1 meter (set by `DistanceController`) using `ComputePathToPose`.
If a new path is computed on the `path` blackboard variable, `FollowPath` will take this `path` and follow it using the server’s default algorithm.

This tree contains:

- No recovery methods
- No retries on failure
- No selected planner or controller algorithms
- No nodes to contextually change settings for optimal performance
- No integration with automatic door, elevator, or other APIs
- No user provided custom BT nodes
- No subtrees for other behaviors like docking, following, etc.
- No use of other types of planners, like complete coverage (where useful)

All of this, and more, can be set and configured for your customized navigation logic in Nav2.
