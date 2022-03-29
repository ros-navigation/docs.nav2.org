.. _bt_cancel_control:

CancelControl
=============

Used to cancel the goals given to the controllers' action server. The server address can be remapped using the ``server_name`` input port.

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

  <CancelControl server_name="FollowPath" server_timeout="10"/>
