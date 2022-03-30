.. _bt_path_longer_on_approach:

PathLongerOnApproach
====================

This node checks if the newly generated global path is significantly larger than the old global path in the user-defined robot's goal proximity and triggers their corresponding children. This allows users to enact special behaviors before a robot attempts to execute a path significantly longer than the prior path when close to a goal (e.g. going around an dynamic obstacle that may just need a few seconds to move out of the way).

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
