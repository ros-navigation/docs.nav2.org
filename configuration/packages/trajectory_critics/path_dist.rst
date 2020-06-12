.. _configuring_dwb_path_dist:

PathDistCritic
==============

Scores a trajectory based on how well it is aligned to the path provided by the global planner.

Parameters
**********

``<name>``: PathDistCritic critic name defined in the **<dwb plugin>.critics** parameter defined in :ref:`dwb_controller`.


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
