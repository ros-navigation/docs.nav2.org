.. _bt_is_battery_charging_condition:

IsBatteryCharging
=================

Checks if the battery is charging by subscribing to a ``sensor_msgs/BatteryState`` topic and checking if the power_supply_status is ``POWER_SUPPLY_STATUS_CHARGING``.
Returns SUCCESS in that case, FAILURE otherwise.

Input Ports
-----------

:battery_topic:
    =============== ===================
    Type            Default
    --------------- -------------------
    string          "/battery_status"
    =============== ===================

    Description
        Topic for battery info.

Example
-------

.. code-block:: xml

    <IsBatteryCharging battery_topic="/battery_status"/>
