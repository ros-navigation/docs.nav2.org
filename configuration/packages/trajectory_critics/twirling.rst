.. _configuring_dwb_twirling:

TwirlingCritic
==============

Prevents holonomic robots from spinning as they make their way to the goal.

Parameters
**********

``<dwb plugin>``: DWB plugin name defined in the **controller_plugin_ids** parameter in :ref:`configuring_controller_server`.

``<name>``: TwirlingCritic critic name defined in the **<dwb plugin>.critics** parameter defined in :ref:`dwb_controller`.

:``<dwb plugin>``.\ ``<name>``.scale:

  ====== =======
  Type   Default
  ------ -------
  double 1.0 
  ====== =======
    
    Description
        Weighed scale for critic.
