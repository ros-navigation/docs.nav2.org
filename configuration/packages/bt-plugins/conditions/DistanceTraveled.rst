.. _bt_distance_traveled_condition:

DistanceTraveled
================

Node that returns success when a configurable distance has been traveled.

Parameters
-----------

:transform_tolerance:

    Defined and declared in :ref:`configuring_bt_navigator`.

Example
^^^^^^^
.. code-block:: yaml

    bt_navigator:
      ros__parameters:
        # other bt_navigator parameters
        transform_tolerance: 0.1

Input Ports
-----------

:distance:

  ====== =======
  Type   Default
  ------ -------
  double 1.0
  ====== =======

  Description
    The distance that must travel before returning success (m).

:global_frame:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    Reference frame. If not provided, uses the BT Navigator's ``global_frame`` setting automatically.

:robot_base_frame:

  ====== ===========
  Type   Default
  ------ -----------
  string N/A
  ====== ===========

  Description
    Robot base frame. If not provided, uses the BT Navigator's ``robot_base_frame`` setting automatically.

Example
-------

.. code-block:: xml

    <DistanceTraveled distance="0.8" global_frame="map" robot_base_frame="base_link"/>
