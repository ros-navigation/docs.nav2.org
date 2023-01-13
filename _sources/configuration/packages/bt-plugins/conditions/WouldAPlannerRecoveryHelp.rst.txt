.. _bt_would_a_planner_recovery_help_condition:

WouldAPlannerRecoveryHelp
=========================

Checks if the active controller server error code is UNKNOWN, NO_VALID_CONTROL, or TIMEOUT.

If the active error code is a match, the node returns ``SUCCESS``. Otherwise, it returns ``FAILURE``. 

Input Port
----------

:error_code:

  ============== =======
  Type           Default
  -------------- -------
  unsigned short  N/A
  ============== =======

  Description
    	The active error code to compare against. This should match the planner server error code. 

Example
-------

.. code-block:: xml

    <WouldAPlannerRecoveryHelp error_code="{compute_path_to_pose_error_code}"/>