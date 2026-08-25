.. _bt_append_goal_pose_to_goals_action:

AppendGoalPoseToGoals
=====================

Appends a goal ``PoseStamped`` to the end of a ``goals`` vector.
May be useful to add in the final task goal pose to a list of goals extracted from Route nodes (or other sources of future goals).

Input Ports
-----------

:goal_pose:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  geometry_msgs/PoseStamped       N/A
  =============================== =======

  Description
        Goal pose to append to the ``goals`` vector.

:input_goals:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  nav_msgs/Goals                  N/A
  =============================== =======

  Description
        Input goals vector to append to.


Output Ports
------------

:output_goals:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  nav_msgs/Goals                  N/A
  =============================== =======

  Description
        Output goals vector appended to.

Example
-------

.. code-block:: xml

    <AppendGoalPoseToGoals goal_pose="{goal}" input_goals="{goal_poses}" output_goals="{goal_poses}"/>
