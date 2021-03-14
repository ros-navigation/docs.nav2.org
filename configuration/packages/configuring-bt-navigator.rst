.. _configuring_bt_navigator:

Behavior-Tree Navigator
#######################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_bt_navigator

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
    Path to the default behavior tree XML description, see :ref:`configuring_behavior_tree_xml` for details on this file.

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

:enable_groot_monitoring:

  ==== =======
  Type Default
  ---- -------
  bool true
  ==== =======

  Description
    Enable live Groot monitoring of the current BT status.
    This is possible due to attaching a ZMQ server and publisher to the active behavior tree.

    **Attention:** Groot will only work after the behavior tree is running, which means that the nav2 stack has to receive a goal first.

:groot_zmq_publisher_port:

  ====== =======
  Type   Default  
  ------ -------
  int    1666   
  ====== =======

  Description
    ZMQ publisher port for the Groot monitor. Used to consecutive publish the current status of the BT as flatbuffer.

:groot_zmq_server_port:

  ====== ======= 
  Type   Default
  ------ -------
  int    1667   
  ====== =======

  Description
    ZMQ server port for the Groot monitor. Used to send the current bt factory + configuration as flatbuffer on an (empty) request by Groot.

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
    Blackboard variable to use to supply the goal to the behavior tree. Should match ports of BT XML file.

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
        enable_groot_monitoring: True
        groot_zmq_publisher_port: 1666
        groot_zmq_server_port: 1667
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
        - nav2_speed_controller_bt_node
        - nav2_recovery_node_bt_node
        - nav2_pipeline_sequence_bt_node
        - nav2_round_robin_node_bt_node
        - nav2_transform_available_condition_bt_node
        - nav2_time_expired_condition_bt_node
        - nav2_distance_traveled_condition_bt_node
        - nav2_single_trigger_bt_node
