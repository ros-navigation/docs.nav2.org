.. _bt_toggle_colllsion_monitor_service:


ToggleCollisionMonitor
======================

Calls the ToggleCollisionMonitor service. Used to toggle the collision monitor on (enabled) and off (disabled).

Input Ports
-----------

:enable:

  ============== =======
  Type           Default
  -------------- -------
  bool           true
  ============== =======

  Description
    	Wether to enable or disable the collision monitor.

:service_name:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A
  ============== =======

  Description
    	Service name.

:server_timeout:

  ============== =======
  Type           Default
  -------------- -------
  double         10
  ============== =======

  Description
    	Server timeout (ms).

Example
-------

.. code-block:: xml

  <ToggleCollisionMonitor enable="false" service_name="collision_monitor/toggle_collision_monitor"/>
