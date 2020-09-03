.. _bt_is_battery_low_condition:

IsBatteryLow
============

Checks if battery is low by subscribing to a ``sensor_msgs/BatteryState`` topic and checking if battery percentage is below a specified minimum value.
Returns SUCCESS when battery percentage is lower than the specified value (in range 0 to 1), FAILURE otherwise.

Example
-------

.. code-block:: xml

    <IsBatteryLow min_battery="0.5" battery_topic="/battery_status"/>
