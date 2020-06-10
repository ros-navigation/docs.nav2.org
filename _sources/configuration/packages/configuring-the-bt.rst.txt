.. _configuring_the_bt:

Behavior-Tree Navigator
#######################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/master/nav2_bt_navigator

The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose task interface. 
It is a Behavior Tree-based implementation of navigation that is intended to allow for flexibility 
in the navigation task and provide a way to easily specify complex robot behaviors, including recovery.

Parameters
**********

:default_bt_xml_filename:

  ====== =======
  Type   Default
  ------ -------
  string N/A   
  ====== =======

  Description
    Path to the default behavior tree XML description.

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
                 "nav2_recovery_node_bt_node",
                 "nav2_pipeline_sequence_bt_node",
                 "nav2_round_robin_node_bt_node",
                 "nav2_transform_available_condition_bt_node"]             
  ============== ==========================================================

  Description
    List of behavior tree node shared libraries.

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

:use_sim_time:

  ==== =======
  Type Default
  ---- -------
  bool false  
  ==== =======

  Description
    Use time provided by simulation.

Example
*******
.. code-block:: yaml

    bt_navigator:
      ros__parameters:
        use_sim_time: true
        global_frame: map
        robot_base_frame: base_link
        transform_tolerance: 0.1
        bt_xml_filename: replace/with/path/to/bt.xml
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
        - nav2_recovery_node_bt_node
        - nav2_pipeline_sequence_bt_node
        - nav2_round_robin_node_bt_node
        - nav2_transform_available_condition_bt_node
