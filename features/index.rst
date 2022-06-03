.. _configuration:

Selected Features
#################

This provides a list of selected features we'd like to highlight in Nav2!
These each describe some important features, important configurations associated with them, and showcases the feature in a hardware demonstration (if applicable). Software design features will be described with diagrams and intent instead.

While we try hard to keep this page up to date with the latest and add any new features, realistically we know we can't.
So please make sure to also check out other parts of this website, including the migration and configuration guides, which may contain more information about these features and newer features available.
If there's something notible you see missing, please consider filing a ticket or letting a maintainer know.

VIDEO HERE 

.. toctree::
   :maxdepth: 1

   feature/autonomous_navigation_framework.rst (framework to implement, with some implementations for common needs, but also OK for drones, etc)
   feature/differential_drive.rst
   feature/omnidirectional_drive.rst
   feature/legged_robot.rst
   feature/ackermann_drive.rst
   feature/rviz_plugin.rst
   feature/goal_and_progress_checkers.rst
   feature/navigate_to_pose.rst
   feature/navigate_through_poses.rst
   feature/follow_dynamic_point.rst
   feature/configurable_nav_logic.rst (but new BT, and new BT per request, groot editing, groot monitoring, 27 BT nodes + examples)
   feature/bt_autonomy.rst (using BT.CPP)
   feature/application_autonomy.rst (using action server)
   feature/waypoint_executors.rst
   feature/keepout_zones.rst
   feature/speed_zones.rst
   feature/high_speed.rst
   feature/multiple_nav_algorithms.rst (selector, hardcode context, abstract application developers, multi-language/compute resource, all the choices we provide)
   feature/modular_servers.rst (same CPU, others, cloud, swap completely, etc; separate controllers/progress/goal)
   feature/quality.rst (doxygen, system tests, unit test coverage, linting, static analysis, use on robots, CI, docker)
   feature/recoveries.rst
   feature/lifecycle_management.rst (lifecycle, bond)
   feature/nav2_actions.rst (useful feedback on state, separated into N servers, interfaces allow multitude of new options like config start/multiple goals)
   feature/shim_controller.rst
   feature/planner_types.rst (holonomic, feasible)
   feature/controller_types.rst (critic, optimization, geometric)
   feature/python_commander.rst

   feature/perception.rst (2d 3d visual(?) depth sonar radar dynamic) -- aspirational
   feature/localization.rst (2d, 3d, visual, gps) -- aspirational
   feature/outdoor.rst (route, height, etc)
   feature/coverage.rst
   feature/map_editing.rst
   feature/semantic.rst (semantic standard, implementation, use for wp/route/anything)
   feature/multistory.rst
   feature/docking.rst
   feature/assisted_teleop.rst
   add videos to the BT XML section
