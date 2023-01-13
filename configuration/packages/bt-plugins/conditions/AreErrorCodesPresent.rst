.. _bt_are_error_codes_present_condition:

AreErrorCodesPresent
====================

Checks the if the provided error code matches any error code within a set.

If the active error code is a match, the node returns ``SUCCESS``. Otherwise, it returns ``FAILURE``. 

Input Ports
-----------

:error_code:

  ============== =======
  Type           Default
  -------------- -------
  unsigned short  N/A
  ============== =======

  Description
    	The active error code to compare against. 

:error_codes_to_check:

  ======================== =======
  Type                     Default
  ------------------------ -------
  std::set<unsigned short> N/A
  ======================== =======

  Description
    	The set of error codes you wish to compare against the active error code. 

Example
-------

Error codes to check are defined in another port. 

.. code-block:: xml

    <AreErrorCodesPresent error_code="{error_code}" error_codes_to_check="{error_codes_to_check}"/>

Error codes to check are defined to be 101, 107 and 119. 

.. code-block:: xml

    <AreErrorCodesPresent error_code="{error_code}" error_codes_to_check="101,107,119"/>
