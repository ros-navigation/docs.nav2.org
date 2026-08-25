.. _bt_follow_object_action:

FollowObject
============

Invokes the FollowObject ROS 2 action server, it will dynamically follow an object while maintaining a defined distance.
The server address can be remapped using the ``server_name`` input port.

Input Ports
-----------
:pose_topic:

  ============== ============
  Type           Default
  -------------- ------------
  string         dynamic_pose
  ============== ============

  Description
        Topic to publish the pose of the object to follow.

:max_duration:

  ============== =======
  Type           Default
  -------------- -------
  double         0.0
  ============== =======

  Description
        The maximum duration to follow the object.

:tracked_frame:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A
  ============== =======

  Description
        Target frame to follow.

:server_name:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
        Action server name.


:server_timeout:

  ============== =======
  Type           Default
  -------------- -------
  double         10
  ============== =======

  Description
        Action server timeout (ms).


Output Ports
------------

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A
  ============== =======

  Description
        Follow object error code. See ``FollowObject`` action for the enumerated set of error code definitions.

:error_msg:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A
  ============== =======

  Description
        Follow object error message. See ``FollowObject`` action for the enumerated set of error code definitions.


Example
-------

.. code-block:: xml

    <FollowObject name="FollowPerson" pose_topic="/person_pose" max_duration="0.0"/>
