.. _bt_cancel_follow_object:

CancelFollowObject
==================

Used to cancel the goals given to the following action server. The server address can be remapped using the ``server_name`` input port.

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

  <CancelFollowObject server_name="follow_object" server_timeout="10"/>
