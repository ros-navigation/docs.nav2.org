.. _bt_cancel_driveonheading:

CancelDriveOnHeading
====================

Used to cancel the drive on heading action that is part of the behavior server. The server address can be remapped using the ``server_name`` input port.

Input Ports
-----------

:service_name:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
      Service name, if not using default of ``drive_on_heading`` due to remapping.


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

  <CancelDriveOnHeading server_name="drive_on_heading" server_timeout="10"/>
