.. _bt_path_longer_on_approach:

PathLongerOnApproach
====================

This node checks if the newly generated global path is significantly larger than the old global path in the user-defined robot's goal proximity and triggers their corresponding children. This particular behavior allows the users to optimize the cycle time by providing with the possibility to avoid their robots to travel through larger corridors, because of a temprorary obstacle closer to the robot's goal proximity. 

Input Ports
-----------

:path:

  ========================== =======
  Type                       Default
  -------------------------- -------
  nav_msgs::msg::Path         N/A  
  ========================== =======

  Description
      Path created by action server. Takes in a blackboard variable, e.g. "{path}".

:prox_len:

  ============= =======
  Type          Default
  ------------- -------
  double         3.0  
  ============= =======

  Description
      Proximity length (m) for the path to be longer on approach.

:length_factor:

  ============= =======
  Type          Default
  ------------- -------
  double         2.0  
  ============= =======

  Description
      Length multiplication factor to check if the path is significantly longer.

Example
-------

.. code-block:: xml

  <PathLongerOnApproach path="{path}" prox_len="3.0" length_factor="2.0">
    <!--Add tree components here--->
  </PathLongerOnApproach>
