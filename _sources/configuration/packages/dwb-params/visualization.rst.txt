.. _dwb_publisher:

Publisher
=========

Parameters
----------

``<dwb plugin>``: DWB plugin name defined in the **controller_plugin_ids** parameter in :ref:`configuring_controller_server`.

:``<dwb plugin>``.publish_evaluation:

  ==== =======
  Type Default
  ---- -------
  bool true      
  ==== =======

  Description
    Whether to publish the local plan evaluation.

:``<dwb plugin>``.publish_global_plan:

  ==== =======
  Type Default
  ---- -------
  bool true      
  ==== =======

  Description
    	Whether to publish the global plan.

:``<dwb plugin>``.publish_transformed_plan:

  ==== =======
  Type Default
  ---- -------
  bool true      
  ==== =======

  Description
    Whether to publish the global plan in the odometry frame.

:``<dwb plugin>``.publish_local_plan:

  ==== =======
  Type Default
  ---- -------
  bool true      
  ==== =======

  Description
    Whether to publish the local planner's plan.

:``<dwb plugin>``.publish_trajectories:

  ==== =======
  Type Default
  ---- -------
  bool true      
  ==== =======

  Description
    	Whether to publish debug trajectories.

:``<dwb plugin>``.publish_cost_grid_pc:

  ==== =======
  Type Default
  ---- -------
  bool false      
  ==== =======

  Description
    Whether to publish the cost grid.

:``<dwb plugin>``.marker_lifetime:

  ============== =======
  Type           Default
  -------------- -------
  double         0.1    
  ============== =======

  Description
    How long for the marker to remain.
