.. bt_actions:

BackUp
#######

Input Ports
**********

:backup_dist:

  ============== =======
  Type           Default
  -------------- -------
  double         -0.15  
  ============== =======

  Description
    	Total distance to backup (m).

:backup_speed:

  ============== =======
  Type           Default
  -------------- -------
  double         0.025 
  ============== =======

  Description
    	Backup speed (m/s).

:server_name:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A  
  ============== =======

  Description
    	Action server name.

:server_timeout:

  ============== =======
  Type           Default
  -------------- -------
  double         10  
  ============== =======

  Description
    	Action server timeout (ms).

Example
*******

.. code-block:: xml

  <BackUp backup_dist="-0.2" backup_speed="0.05" service_name="backup_server" server_timeout="10"/>
    
