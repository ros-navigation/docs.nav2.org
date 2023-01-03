.. _configuring_bt_navigator:

Behavior-Tree Navigator
#######################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_bt_navigator

The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose task interface. 
It is a Behavior Tree-based implementation of navigation that is intended to allow for flexibility 
in the navigation task and provide a way to easily specify complex robot behaviors, including recovery.

Consider checking out the :ref:`groot_introduction` tutorial for using Groot to visualize and modify behavior trees.

Parameters
**********

:navigators:

  ============== ============================================================
  Type           Default
  -------------- ------------------------------------------------------------
  vector<string> {'navigate_to_pose', 'navigate_through_poses'} 
  ============== ============================================================

  Description
    New to Iron: Plugins for navigator types implementing the ``nav2_core::BehaviorTreeNavigator`` interface.
    They implement custom action servers with custom interface definitions and use that data to populate and process behavior tree navigation requests. Plugin classes are defined under the same namespace, see examples below. Defaults correspond to the ``NavigateToPoseNavigator`` and ``NavigateThroughPosesNavigator`` navigators.

:default_nav_to_pose_bt_xml:

  ====== =======
  Type   Default
  ------ -------
  string N/A   
  ====== =======

  Description
    Path to the default behavior tree XML description for ``NavigateToPose``, see :ref:`configuring_behavior_tree_xml` for details on this file.
    Used to be ``default_bt_xml_filename`` pre-Galactic.


:default_nav_through_poses_bt_xml:

  ====== =======
  Type   Default
  ------ -------
  string N/A   
  ====== =======

  Description
    Path to the default behavior tree XML description for ``NavigateThroughPoses``, see :ref:`configuring_behavior_tree_xml` for details on this file. New to Galactic after ``NavigateThroughPoses`` was added. 


:plugin_lib_names:

  ============== ==========================================================
  Type           Default                                                   
  -------------- ----------------------------------------------------------
  vector<string> ["nav2_compute_path_to_pose_action_bt_node", 
                 "nav2_follow_path_action_bt_node",
                 "nav2_back_up_action_bt_node",
                 "nav2_spin_action_bt_node",
                 "nav2_wait_action_bt_node",
                 "nav2_clear_costmap_service_bt_node",
                 "nav2_is_stuck_condition_bt_node",
                 "nav2_goal_reached_condition_bt_node",
                 "nav2_initial_pose_received_condition_bt_node",
                 "nav2_goal_updated_condition_bt_node",
                 "nav2_reinitialize_global_localization_service_bt_node",
                 "nav2_rate_controller_bt_node",
                 "nav2_distance_controller_bt_node",
                 "nav2_speed_controller_bt_node",
                 "nav2_recovery_node_bt_node",
                 "nav2_pipeline_sequence_bt_node",
                 "nav2_round_robin_node_bt_node",
                 "nav2_transform_available_condition_bt_node",
                 "nav2_time_expired_condition_bt_node",
                 "nav2_distance_traveled_condition_bt_node",
                 "nav2_single_trigger_bt_node"]             
  ============== ==========================================================

  Description
    List of behavior tree node shared libraries.

:bt_loop_duration:

  ==== =======
  Type Default
  ---- -------
  int  10
  ==== =======

  Description
    Duration (in milliseconds) for each iteration of BT execution.

:default_server_timeout:

  ==== =======
  Type Default
  ---- -------
  int  20
  ==== =======

  Description
    Default timeout value (in milliseconds) while a BT action node is waiting for acknowledgement from an action server.
    This value will be overwritten for a BT node if the input port "server_timeout" is provided.

:transform_tolerance:

  ====== ======= ======= 
  Type   Default Unit
  ------ ------- -------
  double 0.1     seconds
  ====== ======= =======

  Description
    TF transform tolerance.

:global_frame:

  ====== ======== 
  Type   Default
  ------ --------
  string map    
  ====== ========

  Description
    Reference frame.

:robot_base_frame:

  ====== ========= 
  Type   Default  
  ------ ---------
  string base_link
  ====== =========

  Description
    Path to behavior tree XML description.

:odom_topic:

  ====== =========
  Type   Default
  ------ ---------
  string odom
  ====== =========

  Description
    Topic on which odometry is published

:goal_blackboard_id:

  ====== =======
  Type   Default
  ------ -------
  string "goal"
  ====== =======

  Description
    Blackboard variable to use to supply the goal to the behavior tree for ``NavigateToPose``. Should match ports of BT XML file.

:path_blackboard_id:

  ====== =======
  Type   Default
  ------ -------
  string "path"
  ====== =======

  Description
    Blackboard variable to get the path from the behavior tree for ``NavigateThroughPoses`` feedback. Should match port names of BT XML file.

:goals_blackboard_id:

  ====== =======
  Type   Default
  ------ -------
  string "goals"
  ====== =======

  Description
    Blackboard variable to use to supply the goals to the behavior tree for ``NavigateThroughPoses``. Should match ports of BT XML file.

:use_sim_time:

  ==== =======
  Type Default
  ---- -------
  bool false  
  ==== =======

  Description
    Use time provided by simulation.

:error_code_names:

  ============== ===========================
  Type           Default
  -------------- ---------------------------
  vector<string> ["compute_path_error_code", 
                 "follow_path_error_code"]
  ============== ===========================

  Description
    List of of error codes to compare.

Example
*******
.. code-block:: yaml

    bt_navigator:
      ros__parameters:
        use_sim_time: true
        global_frame: map
        robot_base_frame: base_link
        transform_tolerance: 0.1
        default_nav_to_pose_bt_xml: replace/with/path/to/bt.xml
        default_nav_through_poses_bt_xml: replace/with/path/to/bt.xml
        goal_blackboard_id: goal
        goals_blackboard_id: goals
        path_blackboard_id: path
        navigators: ['navigate_to_pose', 'navigate_through_poses']
        navigate_to_pose:
          plugin: "nav2_bt_navigator/NavigateToPoseNavigator"
        navigate_through_poses:
          plugin: "nav2_bt_navigator/NavigateThroughPosesNavigator"
        plugin_lib_names: 
          - nav2_compute_path_to_pose_action_bt_node
          - nav2_follow_path_action_bt_node
          - nav2_back_up_action_bt_node
          - nav2_spin_action_bt_node
          - nav2_wait_action_bt_node
          - nav2_clear_costmap_service_bt_node
          - nav2_is_stuck_condition_bt_node
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
