.. _dwb_controller:

DWB Controller
==============

Parameters
----------

``<dwb plugin>``: DWB plugin name defined in the **controller_plugin_ids** parameter in :ref:`configuring_controller_server`.

:``<dwb plugin>``.critics:

  ============== =======
  Type           Default
  -------------- -------
  vector<string> N/A    
  ============== =======

  Description
    List of critic plugins to use.

:``<dwb plugin>``.default_critic_namespaces:

  ============== ===============
  Type           Default                                               
  -------------- ---------------
  vector<string> ["dwb_critics"]           
  ============== ===============

  Description
    Namespaces to load critics in.

:``<dwb plugin>``.prune_plan:

  ==== =======
  Type Default
  ---- -------
  bool true   
  ==== =======

  Description
    Whether to prune the path of old, passed points.

:``<dwb plugin>``.shorten_transformed_plan:

  ==== =======
  Type Default
  ---- -------
  bool true   
  ==== =======

  Description
    Determines whether we will pass the full plan on to the critics.

:``<dwb plugin>``.prune_distance:

  ====== =======
  Type   Default
  ------ -------
  double 2.0    
  ====== =======

  Description
    Distance (m) to prune backward until.

:``<dwb plugin>``.forward_prune_distance:

  ====== =======
  Type   Default
  ------ -------
  double 2.0
  ====== =======

  Description
    Distance (m) to prune forward until. If set to ``-1``, it will search the full path for the closest point, in the case of no replanning.

:``<dwb plugin>``.debug_trajectory_details:

  ==== =======
  Type Default                                     
  ---- -------
  bool false  
  ==== =======

  Description
    Publish debug information (on what topic???).

:``<dwb plugin>``.trajectory_generator_name:

  ====== ==========================================
  Type   Default                                               
  ------ ------------------------------------------
  string "dwb_plugins::StandardTrajectoryGenerator"            
  ====== ==========================================

  Description
    Trajectory generator plugin name.

:``<dwb plugin>``.goal_checker_name:

  ============== ================================
  Type           Default                                               
  -------------- --------------------------------
  string         "dwb_plugins::SimpleGoalChecker"           
  ============== ================================

  Description
    Goal checker plugin name.

:``<dwb plugin>``.transform_tolerance:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         0.1        
  ============== =============================

  Description
    TF transform tolerance (s).

:``<dwb plugin>``.short_circuit_trajectory_evaluation:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  bool           true            
  ============== =============================

  Description
    	Stop evaluating scores after best score is found.

:``<dwb plugin>``.path_distance_bias (Legacy):

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         N/A            
  ============== =============================

  Description
    	Old version of PathAlign.scale, use that instead.

:``<dwb plugin>``.goal_distance_bias (Legacy):

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         N/A           
  ============== =============================

  Description
    Old version of GoalAlign.scale, use that instead.

:``<dwb plugin>``.occdist_scale (Legacy):

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         N/A            
  ============== =============================

  Description
    Old version of ObstacleFootprint.scale, use that instead.

:``<dwb plugin>``.max_scaling_factor (Legacy):

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         N/A         
  ============== =============================

  Description
    Old version of ObstacleFootprint.max_scaling_factor, use that instead.

:``<dwb plugin>``.scaling_speed (Legacy):

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         N/A           
  ============== =============================

  Description
    Old version of ObstacleFootprint.scaling_speed, use that instead.

:``<dwb plugin>``.PathAlign.scale:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         32.0    
  ============== =============================

  Description
    Scale for path align critic, overriding local default.

:``<dwb plugin>``.GoalAlign.scale:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         24.0          
  ============== =============================

  Description
    Scale for goal align critic, overriding local default.

:``<dwb plugin>``.PathDist.scale:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         32.0           
  ============== =============================

  Description
    Scale for path distance critic, overriding local default.

:``<dwb plugin>``.GoalDist.scale:

  ============== =============================
  Type           Default                                               
  -------------- -----------------------------
  double         24.0            
  ============== =============================

  Description
    Scale for goal distance critic, overriding local default.
