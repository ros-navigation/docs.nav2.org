.. _humble_migration:

Humble to Iron

Added GPS Waypoint Follower Server
**********************************

`This PR 2627 <https://github.com/ros-planning/navigation2/pull/2814>`_  adds the ``follow_gps_waypoints`` action server in ``nav2_waypoint_follower``. This server accepts a set of GPS goals instead of cartesian goals and provides all the other functionalities available on ``nav2_waypoint_follower``. A new tutorial demonstrating its functionality was also added on `PR 47 on navigation2_tutorials <https://github.com/ros-planning/navigation2_tutorials/pull/47>`_