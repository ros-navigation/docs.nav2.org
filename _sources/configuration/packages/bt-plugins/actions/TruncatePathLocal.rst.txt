.. _bt_truncate_path_local:

TruncatePathLocal
=================

A custom control node, which modifies a path making it shorter. It removes parts of the path which are more distant than specified forward/backward distance around robot

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

:distance_forward:

  ====== ===========
  Type   Default
  ------ -----------
  double 8.0
  ====== ===========

  Description
    	The trimming distance in forward direction.

:distance_backward:

  ====== ===========
  Type   Default
  ------ -----------
  double 4.0
  ====== ===========

  Description
    	The trimming distance in backward direction.

:robot_frame:

  ====== ===========
  Type   Default
  ------ -----------
  string "base_link"
  ====== ===========

  Description
    	Robot base frame id.

:transform_tolerance:

  ====== ===========
  Type   Default
  ------ -----------
  double 0.2
  ====== ===========

  Description
    	Robot pose lookup tolerance.

:pose:

  ========================= ===========
  Type                      Default
  ------------------------- -----------
  geometry_msgs/PoseStamped N/A
  ========================= ===========

  Description
    	Manually specified pose to be used alternatively to current robot pose.

:angular_distance_weight:

  ====== ===========
  Type   Default
  ------ -----------
  double 0.0
  ====== ===========

  Description
    	Weight of angular distance relative to positional distance when finding which path pose is closest to robot. Not applicable on paths without orientations assigned.

:max_robot_pose_search_dist:

  ====== ========
  Type   Default
  ------ --------
  double infinity
  ====== ========

  Description
    	Maximum forward integrated distance along the path (starting from the last detected pose) to bound the search for the closest pose to the robot. When set to infinity (default), whole path is searched every time.

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

  <TruncatePathLocal input_path="{path}" output_path="{path_local}" distance_forward="3.5" distance_backward="2.0" robot_frame="base_link"/>
