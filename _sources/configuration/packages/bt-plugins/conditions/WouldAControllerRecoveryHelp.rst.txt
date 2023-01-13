.. _bt_would_a_controller_recovery_help_condition:

WouldAControllerRecoveryHelp
============================

Checks if the active controller server error code is UNKNOWN, PATIENCE_EXCEEDED, FAILED_TO_MAKE_PROGRESS, or NO_VALID_CONTROL.

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
    	The active error code to compare against. This should match the controller server error code. 

Example
-------

.. code-block:: xml

    <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>