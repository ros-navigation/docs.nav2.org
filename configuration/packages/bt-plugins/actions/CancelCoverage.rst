.. _bt_cancel_coverage:

CancelCoverage
==============

Used to cancel the goals given to the complete coverage action server. The server address can be remapped using the ``server_name`` input port.

Input Ports
-----------

:service_name:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
      Service name.


:server_timeout:

  ====== =======
  Type   Default
  ------ -------
  double 10
  ====== =======

  Description
      Server timeout (ms).

Example
-------

.. code-block:: xml

  <CancelCoverage server_name="compute_complete_coverage" server_timeout="10"/>
