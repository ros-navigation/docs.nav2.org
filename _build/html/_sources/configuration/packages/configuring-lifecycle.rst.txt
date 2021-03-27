.. _configuring_lifecycle_manager:

Lifecycle Manager
#################

Source code on Github_.

.. _Github: https://github.com/ros-planning/navigation2/tree/main/nav2_lifecycle_manager

The Lifecycle Manager module implements the method for handling the lifecycle transition states for the stack in a deterministic way.
It will take in a set of ordered nodes to transition one-by-one into the configurating and activate states to run the stack.
It will then bring down the stack into the finalized state in the opposite order. 
It will also create bond connections with the servrs to ensure they are still up and transition down all nodes if any are non-responsive or crashed.

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

  ==== =======
  Type Default                                                   
  ---- -------
  int  4.0   
  ==== =======

  Description
    Timeout to transition down all lifecycle nodes of this manager if a server is non-responsive, in seconds. Set to ``0`` to deactivate. Recommended to be always larger than 0.3s for all-local node discovery. Note: if a server cleanly exits the manager will immediately be notified.

Example
*******
.. code-block:: yaml

    lifecycle_manager:
      ros__parameters:
        autostart: true
        node_names: ['controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower']
        bond_timeout: 4.0
