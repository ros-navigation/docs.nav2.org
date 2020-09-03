.. _bt_is_battery_low_condition:

IsBatteryLow
============

Checks if battery is low by subscribing to a ``sensor_msgs/BatteryState`` topic and checking if battery voltage/percentage is below a specified minimum value.
By default voltage is used to check for low battery. Set the ``is_percentage`` parameter to `true` to use percentage (in range 0 to 1).
Returns SUCCESS when battery voltage/percentage is lower than the specified value, FAILURE otherwise.

Example
-------

.. code-block:: xml

    <IsBatteryLow min_battery="5.0" battery_topic="/battery_status" is_percentage="false"/>
