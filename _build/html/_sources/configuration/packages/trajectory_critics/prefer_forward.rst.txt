.. _configuring_dwb_prefer_forward:

PreferForwardCritic
===================

Scores trajectories that move the robot forwards more highly.

Parameters
**********

``<dwb plugin>``: DWB plugin name defined in the **controller_plugin_ids** parameter in :ref:`configuring_controller_server`.

``<name>``: PreferForwardCritic critic name defined in the **<dwb plugin>.critics** parameter defined in :ref:`dwb_controller`.


:``<dwb plugin>``.\ ``<name>``.penalty:

  ====== =======
  Type   Default
  ------ -------
  double 1.0 
  ====== =======
    
    Description
        Penalty to apply to backward motion.

:``<dwb plugin>``.\ ``<name>``.strafe_x:

  ====== =======
  Type   Default
  ------ -------
  double 0.1 
  ====== =======
    
    Description
        	Minimum X velocity before penalty.

:``<dwb plugin>``.\ ``<name>``.strafe_theta:

  ====== =======
  Type   Default
  ------ -------
  double 0.2 
  ====== =======
    
    Description
        Minimum angular velocity before applying penalty.

:``<dwb plugin>``.\ ``<name>``.theta_scale:

  ====== =======
  Type   Default
  ------ -------
  double 10.0 
  ====== =======
    
    Description
        Weight for angular velocity component.

:``<dwb plugin>``.\ ``<name>``.scale:

  ====== =======
  Type   Default
  ------ -------
  double 1.0 
  ====== =======
    
    Description
        Weighed scale for critic.
