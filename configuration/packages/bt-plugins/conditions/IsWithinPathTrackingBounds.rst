.. _bt_is_within_path_tracking_bounds_condition:

IsWithinPathTrackingBounds
==========================

Checks if the robot is within determined tracking error bounds during path following. This condition node subscribes to tracking feedback messages and verifies that tracking error stays within determined left and right boundaries.

The node differentiates between left-side errors (positive values) and right-side errors (negative values), allowing asymmetric bounds to be configured for each side.

Input Ports
-----------

:max_error_left:

  ====== =======
  Type   Default
  ------ -------
  double N/A
  ====== =======

  Description
    Maximum allowable tracking error (m) on the left side of the path before returning FAILURE. Must be a positive value.

:max_error_right:

  ====== =======
  Type   Default
  ------ -------
  double N/A
  ====== =======

  Description
    Maximum allowable tracking error (m) on the right side of the path before returning FAILURE. Must be a positive value.

Example
-------

.. code-block:: xml

    <IsWithinPathTrackingBounds max_error_left="0.5" max_error_right="0.5"/>

This example checks that the robot stays within 0.5 meters of the path on both sides.
