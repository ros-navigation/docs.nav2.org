.. _bt_is_battery_low_condition:

IsBatteryLow
============

Checks if battery is low by subscribing to a ``sensor_msgs/BatteryState`` topic and checking if battery percentage is below a specified minimum value.
Returns SUCCESS when battery percentage is lower than the specified value, FAILURE otherwise.

Example
-------

.. code-block:: xml

    <IsBatteryLow min_battery="50.0" battery_topic="/battery_status"/>
