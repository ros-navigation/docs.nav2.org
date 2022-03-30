.. _bt_cancel_spin:

CancelSpin
==========

Used to cancel the spin action that is part of the behavior server. The server address can be remapped using the ``server_name`` input port.

Input Ports
-----------

:service_name:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
      Service name, if not using default of ``spin`` due to remapping.


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

  <CancelSpin server_name="Spin" server_timeout="10"/>
