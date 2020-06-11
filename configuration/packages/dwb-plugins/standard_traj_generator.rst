.. _configuring_dwb_stand_traj_gen_plugin:

standard_traj_generator
=======================

Parameters
**********

:``<dwb plugin>``.sim_time:

  ====== =======
  Type   Default
  ------ -------
  double 1.7
  ====== =======
    
    Description
        Time to simulate ahead by (s).

:``<dwb plugin>``.discretize_by_time:

  ==== =======
  Type Default
  ---- -------
  bool false
  ==== =======
    
    Description
        If true, forward simulate by time. If False, forward simulate by linear and angular granularity.

:``<dwb plugin>``.time_granularity:

  ====== =======
  Type   Default
  ------ -------
  double 0.5
  ====== =======
    
    Description
        Time ahead to project.

:``<dwb plugin>``.linear_granularity:

  ====== =======
  Type   Default
  ------ -------
  double 0.5
  ====== =======
    
    Description
        Linear distance forward to project.

:``<dwb plugin>``.angular_granularity:

  ====== =======
  Type   Default
  ------ -------
  double 0.025
  ====== =======
    
    Description
        Angular distance to project.

:``<dwb plugin>``.include_last_point:

  ==== =======
  Type Default
  ---- -------
  bool true
  ==== =======
    
    Description
        Whether to include the last pose in the trajectory.

Example
*******

.. code-block:: yaml