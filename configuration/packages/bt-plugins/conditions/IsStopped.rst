.. _bt_is_stopped_condition:

IsStopped
=========

BT node that tracks robot odometry and returns SUCCESS if robot is considered stopped for long enough,
RUNNING if stopped but not for long enough and FAILURE otherwise

Input Port
----------

:velocity_threshold:

  ======= =======
  Type    Default
  ------- -------
  double   0.01
  ======= =======

  Description
    	Velocity threshold below which robot is considered stopped

:duration_stopped:

  ========  =======
  Type      Default
  --------  -------
  int (ms)  1000         
  ========  =======

  Description
    	Duration (ms) the velocity must remain below the threshold 

Example
-------

.. code-block:: xml

    <IsStopped velocity_threshold="0.01" duration_stopped="1000"/>