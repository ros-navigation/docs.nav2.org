.. _bt_truncate_path:

TruncatePath
============

A custom control node, which modifies a path making it shorter. It removes parts of the path closer than a distance to the goal pose. The resulting last pose of the path orientates the robot to the original goal pose.

Input Ports
-----------

:input_path:

  ============= =======
  Type          Default
  ------------- -------
  nav_msgs/Path N/A
  ============= =======

  Description
      The original path to be truncated.

:distance:

  ====== ===========
  Type   Default
  ------ -----------
  double 1.0
  ====== ===========

  Description
    	The distance to the original goal for truncating the path.

Ouput Ports
-----------

:output_path:

  ============= =======
  Type          Default
  ------------- -------
  nav_msgs/Path N/A
  ============= =======

  Description
    	The resulting truncated path.

Example
-------

.. code-block:: xml

  <TruncatePath distance="1.0" input_path="{path}" output_path="{truncated_path}"/>
