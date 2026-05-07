.. _bt_is_within_path_tracking_bounds_condition:

IsWithinPathTrackingBounds
==========================

Checks if the robot is within determined tracking error bounds during path following.

Input Ports
-----------

:max_error_left:

  ====== =======
  Type   Default
  ------ -------
  double 0.5
  ====== =======

  Description
    Maximum allowable tracking error (m) on the left side of the path before returning FAILURE. Must be a positive value.

:max_error_right:

  ====== =======
  Type   Default
  ------ -------
  double 0.5
  ====== =======

  Description
    Maximum allowable tracking error (m) on the right side of the path before returning FAILURE. Must be a positive value.

:max_error_heading:

    ====== =======
    Type   Default
    ------ -------
    double 3.14
    ====== =======

    Description
      Maximum allowable heading error (rad) before returning FAILURE. Must be a positive value.

:tracking_feedback:

    ==================================== ========
    Type                                 Default
    ------------------------------------ --------
    nav2_msgs::msg::PathTrackingFeedback N/A
    ==================================== ========

    Description
      Generally, the feedback message from the controller server, which contains the current tracking error information. Though, it may be populated by another source or topic if need be.

Example
-------

.. code-block:: xml

    <IsWithinPathTrackingBounds max_error_left="0.5" max_error_right="0.5" max_error_heading="3.14" tracking_feedback="{tracking_feedback}"/>
