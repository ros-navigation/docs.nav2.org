.. _configuring_dwb_rotate_to_goal:

rotate_to_goal
==============

**CONFIRM CONTAINS <name>**

Parameters
**********

:``<dwb plugin>``.\ ``<name>``.xy_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25 
  ====== =======
    
    Description
        Tolerance to meet goal completion criteria (m).

:``<dwb plugin>``.\ ``<name>``.trans_stopped_velocity:

  ====== =======
  Type   Default
  ------ -------
  double 0.25 
  ====== =======
    
    Description
        Velocity below is considered to be stopped at tolerance met (rad/s).

:``<dwb plugin>``.\ ``<name>``.slowing_factor:

  ====== =======
  Type   Default
  ------ -------
  double 5.0 
  ====== =======
    
    Description
       	Factor to slow robot motion by while rotating to goal.

:``<dwb plugin>``.\ ``<name>``.lookahead_time:

  ====== =======
  Type   Default
  ------ -------
  double -1 
  ====== =======
    
    Description
        If > 0, amount of time to look forward for a collision for..

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