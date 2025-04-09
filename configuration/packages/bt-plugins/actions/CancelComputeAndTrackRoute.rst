.. _bt_cancel_compute_and_track_route:

CancelComputeAndTrackRoute
==========================

Used to cancel the compute and track route action that is part of the behavior server. The server address can be remapped using the ``server_name`` input port.

Input Ports
-----------

:service_name:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
      Service name, if not using default of ``compute_and_track_route`` due to remapping.


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

  <CancelComputeAndTrackRoute server_name="compute_and_track_route" server_timeout="10"/>
