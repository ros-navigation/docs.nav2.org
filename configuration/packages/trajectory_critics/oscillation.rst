.. _configuring_dwb_oscillation:

oscillation
===========

Parameters
**********

:``<dwb plugin>``.\ ``<name>``.oscillation_reset_dist:

  ====== =======
  Type   Default
  ------ -------
  double 0.05  
  ====== =======
    
    Description
        Minimum distance to move to reset oscillation watchdog (m).

:``<dwb plugin>``.\ ``<name>``.oscillation_reset_angle:

  ====== =======
  Type   Default
  ------ -------
  double 0.2  
  ====== =======
    
    Description
        Minimum angular distance to move to reset watchdog (rad).

:``<dwb plugin>``.\ ``<name>``.oscillation_reset_time:

  ====== =======
  Type   Default
  ------ -------
  double -1  
  ====== =======
    
    Description
        Duration when a reset may be called. If -1, cannot be reset..


:``<dwb plugin>``.\ ``<name>``.x_only_threshold:

  ====== =======
  Type   Default
  ------ -------
  double 0.05  
  ====== =======
    
    Description
        Threshold to check in the X velocity direction.

:``<dwb plugin>``.\ ``<name>``.scale:

  ====== =======
  Type   Default
  ------ -------
  double 1.0 
  ====== =======
    
    Description
        Weighed scale for critic.

Example
*******

.. code-block:: yaml