.. _configuring_lifecycle_manager:

Lifecycle Manager
#################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_lifecycle_manager

The Lifecycle Manager module implements the method for handling the lifecycle transition states for the stack in a deterministic way.
It will take in a set of ordered nodes to transition one-by-one into the configurating and activate states to run the stack.
It will then bring down the stack into the finalized state in the opposite order.
It will also create bond connections with the servers to ensure they are still up and transition down all nodes if any are non-responsive or crashed.

Parameters
**********

:node_names:

  ============== =======
  Type           Default
  -------------- -------
  vector<string>  N/A
  ============== =======

  Description
    Ordered list of node names to bringup through lifecycle transition.

:autostart:

  ==== =======
  Type Default
  ---- -------
  bool false
  ==== =======

  Description
    Whether to transition nodes to active state on startup.

:bond_timeout:

  ====== =======
  Type   Default
  ------ -------
  double 4.0
  ====== =======

  Description
    Timeout to transition down all lifecycle nodes of this manager if a server is non-responsive, in seconds. Set to ``0`` to deactivate. Recommended to be always larger than 0.3s for all-local node discovery. Note: if a server cleanly exits the manager will immediately be notified.

:attempt_respawn_reconnection:

  ==== =======
  Type Default
  ---- -------
  bool true
  ==== =======

  Description
    Whether to try to reconnect to servers that go down, presumably because respawn is set to ``true`` to re-create crashed nodes. While default to ``true``, reconnections will not be made unless respawn is set to true in your launch files or your watchdog systems will bring up the server externally.

:bond_respawn_max_duration:

  ====== =======
  Type   Default
  ------ -------
  double  10.0
  ====== =======

  Description
    When a server crashes or becomes non-responsive, the lifecycle manager will bring down all nodes for safety. This is the duration of which the lifecycle manager will attempt to reconnect with the failed server(s) during to recover and re-activate the system. If this passes, it will stop attempts and will require a manual re-activation once the problem is manually resolved. Units: seconds.

Example
*******
.. code-block:: yaml

    lifecycle_manager:
      ros__parameters:
        autostart: true
        node_names: ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower']
        bond_timeout: 4.0
        attempt_respawn_reconnection: true
        bond_respawn_max_duration: 10.0
