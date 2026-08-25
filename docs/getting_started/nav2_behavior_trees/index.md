# Nav2 Behavior Trees { #nav2-behavior-trees }

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
If a new path is computed on the `path` blackboard variable, `FollowPath` will take this `path` and follow it using the server's default algorithm.

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

<div class="grid cards bottom-align" markdown>

- :material-book-open-variant: **Introduction To Nav2 Specific Nodes**

    ---
    Overview of Nav2's custom BT action, condition, control, and decorator nodes.

    [:octicons-arrow-right-24: Go][introduction-to-nav2-specific-nodes]

- :material-magnify: **Detailed Behavior Tree Walkthrough**

    ---
    Step-by-step walkthrough of the default navigate-to-pose behavior tree.

    [:octicons-arrow-right-24: Go][detailed-behavior-tree-walkthrough]

</div>

<span class="section-title">Provided Nav2 Behavior Trees</span>

<div class="grid cards bottom-align" markdown>

- :material-map-marker: **Navigate To Pose**

    ---
    Standard point-to-point navigation with recovery.

    [:octicons-arrow-right-24: Go][nav2-bt-navigate-to-pose]

- :material-map-marker-path: **Navigate Through Poses**

    ---
    Multi-waypoint navigation through a sequence of poses.

    [:octicons-arrow-right-24: Go][nav2-bt-navigate-through-poses]

- :material-pause-circle: **Navigate To Pose and Pause Near Goal-Obstacle**

    ---
    Pause and wait for obstacles near the goal to clear before detouring.

    [:octicons-arrow-right-24: Go][navigate-to-pose-and-pause-near-goal-obstacle]

- :material-refresh: **Navigate To Pose With Consistent Replanning And If Path Becomes Invalid**

    ---
    Continuous replanning with path validity monitoring.

    [:octicons-arrow-right-24: Go][navigate-to-pose-with-consistent-replanning-and-if-path-becomes-invalid]

- :material-graph: **Navigate on Route Graph with Recovery**

    ---
    Route-graph-based navigation with failure recovery.

    [:octicons-arrow-right-24: Go][navigate-on-route-graph-with-recovery]

- :material-target: **Follow Dynamic Point**

    ---
    Track and follow a moving target in real time.

    [:octicons-arrow-right-24: Go][follow-dynamic-point]

- :material-compass: **Odometry Calibration**

    ---
    BT for automated odometry calibration routines.

    [:octicons-arrow-right-24: Go][odometry-calibration]

</div>
