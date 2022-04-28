.. _bt_path_expiring_timer_condition:

PathExpiringTimer
=================

Checks if the timer has expired. Returns success if the timer has expired, otherwise it returns failure.
The timer will reset if the path gets updated.

Example
-------

.. code-block:: xml

    <PathExpiringTimer seconds="15" path="{path}"/>
