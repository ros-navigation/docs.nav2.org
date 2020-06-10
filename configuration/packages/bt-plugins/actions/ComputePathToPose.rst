.. bt_actions:

ComputePathToPose
#################

Input Ports
-----------

:goal:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A  
  ============== =======

  Description
    	Goal pose.

:planner_id:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A  
  ============== =======

  Description
    	Mapped name to the planner plugin type to use, e.g. GridBased.

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
  
Output Ports
------------

:path:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A  
  ============== =======

  Description
    	Path created by action server.

Example
*******

.. codeblock:: xml
    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
