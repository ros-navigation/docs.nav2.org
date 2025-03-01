.. _bt_is_battery_low_condition:

IsBatteryLow
============

Checks if battery is low by subscribing to a ``sensor_msgs/BatteryState`` topic and checking if battery percentage/voltage is below a specified minimum value.
By default percentage (in range 0 to 1) is used to check for low battery. Set the ``is_voltage`` parameter to `true` to use voltage.
Returns SUCCESS when battery percentage/voltage is lower than the specified value, FAILURE otherwise.

Input Ports
-----------

:min_battery:
    ====== =======
    Type   Default
    ------ -------
    double 0.0
    ====== =======

    Description
        Min battery percentage or voltage before triggering.

:battery_topic:
    =============== ===================
    Type            Default
    --------------- -------------------
    string          "/battery_status"
    =============== ===================
    
    Description
        Topic for battery info.

:is_voltage:
    ====== =======
    Type   Default
    ------ -------
    bool   false
    ====== =======
    
    Description
        If true voltage will be used to check for low battery.

Example
-------

.. code-block:: xml

    <IsBatteryLow min_battery="0.5" battery_topic="/battery_status" is_voltage="false"/>
