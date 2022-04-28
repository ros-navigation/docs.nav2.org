.. _bt_speed_controller:

SpeedController
==================

A node that controls the tick rate for its child based on current robot speed.
The maximum and minimum replanning rates can be supplied to the node as parameters along with maximum and minimum speed.
The node returns RUNNING when it is not ticking its child. Currently, in the navigation 
stack, the ``SpeedController`` is used to adjust the rate at which the ``ComputePathToPose`` and ``GoalReached`` nodes are ticked.

Input Ports
-----------

:min_rate:

  ====== =======
  Type   Default
  ------ -------
  double  0.1
  ====== =======

  Description
      The minimum rate at which child node can be ticked (hz).

:max_rate:

  ====== =======
  Type   Default
  ------ -------
  double  1.0
  ====== =======

  Description
      The maximum rate at which child node can be ticked (hz).

:min_speed:

  ====== =======
  Type   Default
  ------ -------
  double  0.0
  ====== =======

  Description
      The minimum robot speed below which the child node is ticked at minimum rate (m/s).

:max_speed:

  ====== =======
  Type   Default
  ------ -------
  double  0.5
  ====== =======

  Description
      The maximum robot speed above which the child node is ticked at maximum rate (m/s).

:filter_duration:

  ====== =======
  Type   Default
  ------ -------
  double  0.3
  ====== =======

  Description
      Duration (secs) over which robot velocity should be smoothed.

Example
-------

.. code-block:: xml

  <SpeedController min_rate="0.1" max_rate="1.0" min_speed="0.0" max_speed="0.5" filter_duration="0.3">
    <!--Add tree components here--->
  </SpeedController>
