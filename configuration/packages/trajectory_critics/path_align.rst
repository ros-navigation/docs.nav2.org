.. _configuring_dwb_path_align:

PathAlignCritic
===============

Scores a trajectory based on how well it is aligned to the path provided by the global planner.

Parameters
**********

``<name>``: PathAlignCritic critic name defined in the **<dwb plugin>.critics** parameter defined in :ref:`dwb_controller`.


:``<dwb plugin>``.\ ``<name>``.forward_point_distance:

  ====== =======
  Type   Default
  ------ -------
  double 0.325 
  ====== =======
    
    Description
        Point in front of robot to look ahead to compute angular change from.

:``<dwb plugin>``.\ ``<name>``.aggregation_type:

  ====== =======
  Type   Default
  ------ -------
  string "last" 
  ====== =======
    
    Description
        last, sum, or product combination methods.

:``<dwb plugin>``.\ ``<name>``.scale:

  ====== =======
  Type   Default
  ------ -------
  double 1.0 
  ====== =======
    
    Description
        Weighed scale for critic.
