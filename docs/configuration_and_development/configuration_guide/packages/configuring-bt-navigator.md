# Behavior-Tree Navigator { #behavior-tree-navigator }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator).

The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose, NavigateThroughPoses, and other task interfaces.
It is a Behavior Tree-based implementation of navigation that is intended to allow for flexibility
in the navigation task and provide a way to easily specify complex robot behaviors, including recovery.

Consider checking out the [Groot Tutorials][groot-tutorials] tutorial for using Groot to visualize and modify behavior trees.

Make sure to review all parameters for non-default navigator plugins not discussed on this page (e.g. `CoverageNavigator` or custom additions).

## Parameters

### **`navigators`**

| Type             | Default                                        |
|------------------|------------------------------------------------|
| `vector<string>` | {'navigate_to_pose', 'navigate_through_poses'} |

Description
:   New to Iron: Plugins for navigator types implementing the `nav2_core::BehaviorTreeNavigator` interface.
    They implement custom action servers with custom interface definitions and use that data to populate and process behavior tree navigation requests. Plugin classes are defined under the same namespace, see examples below. Defaults correspond to the `NavigateToPoseNavigator` and `NavigateThroughPosesNavigator` navigators.

### **`default_nav_to_pose_bt_xml`**

| Type     | Default |
|----------|---------|
| `string` | N/A     |

Description
:   Path to the default behavior tree XML description for `NavigateToPose`, 
    see [Behavior Tree XML Nodes][behavior-tree-xml-nodes] for details on this file.
    This parameter used to be `default_bt_xml_filename` pre-Galactic.
    You can use substitution to specify file path like `$(find-pkg-share my_package)/behavior_tree/my_nav_to_pose_bt.xml`. However, if left empty, the default behavior tree XML will be loaded from the `nav2_bt_navigator` package.

### **`default_nav_through_poses_bt_xml`**

| Type     | Default |
|----------|---------|
| `string` | N/A     |

Description
:   Path to the default behavior tree XML description for `NavigateThroughPoses`, 
    see [Behavior Tree XML Nodes][behavior-tree-xml-nodes] for details on this file. New to Galactic after `NavigateThroughPoses` was added.
    You can use substitution to specify file path like `$(find-pkg-share my_package)/behavior_tree/my_nav_through_poses_bt.xml`. However, if left empty, the default behavior tree XML will be loaded from the `nav2_bt_navigator` package.

### **`always_reload_bt_xml`**

| Type   | Default |
|--------|---------|
| `bool` | false   |

Description
:   Always load the requested behavior tree XML description, regardless of the name of the currently active XML.

### **`plugin_lib_names`**

| Type             | Default |
|------------------|---------|
| `vector<string>` | [""]    |

Description
:   List of behavior tree node shared libraries. All Nav2 BT libraries are automatically included for you, so this only needs to include your new custom plugins (new to Jazzy).

### **`bt_loop_duration`**

| Type  | Default |
|-------|---------|
| `int` | 10      |

Description
:   Duration (in milliseconds) for each iteration of BT execution.

### **`filter_duration`**

| Type     | Default |
|----------|---------|
| `double` | 0.3     |

Description
:   Duration (secs) over which robot velocity should be smoothed.

### **`default_server_timeout`**

| Type  | Default |
|-------|---------|
| `int` | 20      |

Description
:   Default timeout value (in milliseconds) for a BT action node to wait for acknowledgement from an action server.
    This value will be overwritten for a BT node if the input port "server_timeout" is provided.

### **`default_cancel_timeout`**

| Type  | Default |
|-------|---------|
| `int` | 50      |

Description
:   Default timeout (in milliseconds) for BT action node cancellation requests during node halt.
    This value will be overwritten for a BT node if the input port "cancel_timeout" is provided.

### **`wait_for_service_timeout`**

| Type  | Default |
|-------|---------|
| `int` | 1000    |

Description
:   Default timeout value (in milliseconds) for an Action or Service BT nodes to wait for acknowledgement from an service or action server on BT initialization (e.g. `wait_for_action_server(timeout)`).
    This value will be overwritten for a BT node if the input port "wait_for_service_timeout" is provided.

### **`action_server_result_timeout`**

| Type     | Default | Unit    |
|----------|---------|---------|
| `double` | 900.0   | seconds |

Description
:   The timeout value (in seconds) for action servers to discard a goal handle if a result has not been produced. This used to default to 15 minutes in rcl but was changed to 10 seconds in this [PR #1012]( https://github.com/ros2/rcl/pull/1012), which may be less than some actions in Nav2 take to run. For most applications, this should not need to be adjusted as long as the actions within the server do not exceed this deadline. This issue has been raised with OSRF to find another solution to avoid active goal timeouts for bookkeeping, so this is a semi-temporary workaround.

### **`transform_tolerance`**

| Type     | Default | Unit    |
|----------|---------|---------|
| `double` | 0.1     | seconds |

Description
:   TF transform tolerance.

### **`global_frame`**

| Type     | Default |
|----------|---------|
| `string` | map     |

Description
:   Reference frame.

### **`robot_base_frame`**

| Type     | Default   |
|----------|-----------|
| `string` | base_link |

Description
:   Robot base frame.

### **`odom_topic`**

| Type     | Default |
|----------|---------|
| `string` | odom    |

Description
:   Topic on which odometry is published

### **`goal_blackboard_id`**

| Type     | Default |
|----------|---------|
| `string` | "goal"  |

Description
:   Blackboard variable to use to supply the goal to the behavior tree for `NavigateToPose`. Should match ports of BT XML file.

### **`path_blackboard_id`**

| Type     | Default |
|----------|---------|
| `string` | "path"  |

Description
:   Blackboard variable to get the path from the behavior tree for `NavigateThroughPoses` feedback. Should match port names of BT XML file.

### **`goals_blackboard_id`**

| Type     | Default |
|----------|---------|
| `string` | "goals" |

Description
:   Blackboard variable to use to supply the goals to the behavior tree for `NavigateThroughPoses`. Should match ports of BT XML file.

### **`error_code_names`**

| Type             | Default                                                   |
|------------------|-----------------------------------------------------------|
| `vector<string>` | ["compute_path_error_code",<br> "follow_path_error_code"] |

Description
:   List of of error codes to compare.

### **`bond_heartbeat_period`**

| Type     | Default |
|----------|---------|
| `double` | 0.1     |

Description
:   The lifecycle node bond mechanism publishing period (on the /bond topic). Disabled if inferior or equal to 0.0.

## NavigateToPose Parameters

### **`<navigate_to_pose_name>.tracking_feedback_blackboard_id`**

| Type     | Default             |
|----------|---------------------|
| `string` | "tracking_feedback" |

Description
:   Blackboard variable to get the tracking feedback from the behavior tree for `NavigateToPose` feedback. Should match port names of BT XML file.

### **`<navigate_to_pose_name>.enable_groot_monitoring`**

| Type   | Default |
|--------|---------|
| `bool` | False   |

Description
:   Whether to enable Groot2 monitoring for this navigator.

### **`<navigate_to_pose_name>.groot_server_port`**

| Type  | Default |
|-------|---------|
| `int` | 1667    |

Description
:   The port number for the Groot2 server. Note: In Groot2, you only need to specify the server port value, not the publisher port, as it is always the server port +1. Therefore, in this case, to use another navigator, the next available port would be 1669.

### **`<navigate_to_pose_name>.search_window`**

| Type     | Default |
|----------|---------|
| `double` | 2.0     |

Description
:   How far (in meters) along the path the searching algorithm will look for the closest point.

## NavigateThroughPoses Parameters

### **`<navigate_through_poses>.tracking_feedback_blackboard_id`**

| Type     | Default             |
|----------|---------------------|
| `string` | "tracking_feedback" |

Description
:   Blackboard variable to get the tracking feedback from the behavior tree for `NavigateThroughPoses` feedback. Should match port names of BT XML file.

### **`<navigate_through_poses>.enable_groot_monitoring`**

| Type   | Default |
|--------|---------|
| `bool` | False   |

Description
:   Whether to enable Groot2 monitoring for this navigator.

### **`<navigate_through_poses>.groot_server_port`**

| Type  | Default |
|-------|---------|
| `int` | 1669    |

Description
:   The port number for the Groot2 server. Note: In Groot2, you only need to specify the server port value, not the publisher port, as it is always the server port +1. Therefore, in this case, to use another navigator, the next available port would be 1671.

### **`<navigate_through_poses>.search_window`**

| Type     | Default |
|----------|---------|
| `double` | 2.0     |

Description
:   How far (in meters) along the path the searching algorithm will look for the closest point.

## Example

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    filter_duration: 0.3
    default_server_timeout: 20
    default_cancel_timeout: 50
    default_nav_to_pose_bt_xml: replace/with/path/to/bt.xml # or $(find-pkg-share my_package)/behavior_tree/my_nav_to_pose_bt.xml
    default_nav_through_poses_bt_xml: replace/with/path/to/bt.xml # or $(find-pkg-share my_package)/behavior_tree/my_nav_through_poses_bt.xml
    always_reload_bt_xml: false
    goal_blackboard_id: goal
    goals_blackboard_id: goals
    path_blackboard_id: path
    navigators: ['navigate_to_pose', 'navigate_through_poses']
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator" # In Iron and older versions, "/" was used instead of "::"
      enable_groot_monitoring: false
      groot_server_port: 1667
      search_window: 2.0
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator" # In Iron and older versions, "/" was used instead of "::"
      enable_groot_monitoring: false
      groot_server_port: 1669
      search_window: 2.0
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_is_stopped_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
    error_code_names:
      - compute_path_error_code
      - follow_path_error_code
      # - smoother_error_code, navigate_to_pose_error_code, navigate_through_poses_error_code, etc
```
