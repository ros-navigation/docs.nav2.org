.. bt_actions:

FollowPath
==========

Input Ports
-----------

:path:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	Path to follow.

:controller_id:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	Mapped name of the controller plugin type to use, e.g. FollowPath.

:server_name:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

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
-------

.. code-block:: xml
    <FollowPath path="{path}" controller_id="FollowPath"/>
