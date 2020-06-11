.. _configuring_dwb_simple_goal_checker_plugin:

simple_goal_checker
===================

Parameters
**********

:``<dwb plugin>``.xy_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======
    
    Description
        Tolerance to meet goal completion criteria (m).

:``<dwb plugin>``.yaw_goal_tolerance:

  ====== =======
  Type   Default
  ------ -------
  double 0.25
  ====== =======
    
    Description
        Tolerance to meet goal completion criteria (rad).

:``<dwb plugin>``.stateful:

  ==== =======
  Type Default
  ---- -------
  bool true 
  ==== =======
    
    Description
        Whether to check for XY position tolerance after rotating to goal orientation in case of minor localization changes.

Example
*******

.. code-block:: yaml