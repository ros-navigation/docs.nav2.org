.. _bt_cancel_wait:

CancelWait
==========

Used to cancel the wait action that is part of the behavior server. The server address can be remapped using the ``server_name`` input port.

Input Ports
-----------

:service_name:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
      Service name, if not using default of ``wait`` due to remapping.


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

  <CancelWait server_name="Wait" server_timeout="10"/>
