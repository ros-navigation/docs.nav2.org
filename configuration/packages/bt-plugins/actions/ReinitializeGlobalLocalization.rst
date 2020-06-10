.. bt_actions:

ReinitializeGlobalLocalization
##############################

Input Ports
-----------

:service_name:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	Server name.


:server_timeout:

  ====== =======
  Type   Default
  ------ -------
  double 10  
  ====== =======

  Description
    	Server timeout (ms).

Example
*******

.. code-block:: xml
   <ReinitializeGlobalLocalization service_name="local_costmap/clear_entirely_local_costmap"/>
