.. _bt_path_expiring_timer_condition:

PathExpiringTimer
=================

Checks if the timer has expired. Returns success if the timer has expired, otherwise it returns failure.
The timer will reset if the path gets updated.

Input Ports
-----------

:seconds:
  ====== =======
  Type   Default
  ------ -------
  double 1.0
  ====== =======

  Description
    Time to check if expired.

:path:
  ==================================== =======
  Type                                 Default
  ------------------------------------ -------
  nav_msgs::msg::Path                  N/A
  ==================================== =======

  Description
    Check if path has been updated to enable timer reset.


Example
-------

.. code-block:: xml

    <PathExpiringTimer seconds="15" path="{path}"/>
