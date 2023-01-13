.. _bt_would_a_smoother_recovery_help_condition:

WouldASmootherRecoveryHelp
==========================

Checks if the active controller server error code is UNKNOWN, TIMEOUT, FAILED_TO_SMOOTH_PATH, or SMOOTHED_PATH_IN_COLLISION.

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
    	The active error code to compare against. This should match the smoother server error code. 

Example
-------

.. code-block:: xml

    <WouldASmootherRecoveryHelp error_code="{smoother_error_code}"/>