.. _bt_is_path_valid_condition:

IsPathValid
===========

Checks to see if the global path is valid. If there is a
obstacle along the path, the condition returns FAILURE, otherwise
it returns SUCCESS.

Input Ports
-----------

:service_timeout:

  ====== =======
  Type   Default
  ------ -------
  double 20.0
  ====== =======

  Description
    Service response timeout (ms).

:path:

  ==================================== =======
  Type                                 Default
  ------------------------------------ -------
  nav_msgs::msg::Path                  N/A  
  ==================================== =======

  Description
    The global path to check for validity.

:max_cost:

  ============== ==========
  Type           Default
  -------------- ----------
  unsigned int   253
  ============== ==========

  Description
    The maximum allowable cost for the path to be considered valid.

:consider_unknown_as_obstacle:

  ====== =======
  Type   Default
  ------ -------
  bool   false  
  ====== =======

  Description
    Whether to consider unknown cost (255) as obstacle.


Example
-------

.. code-block:: xml

    <IsPathValid server_timeout="10" path="{path}" max_cost="100" consider_unknown_as_obstacle="false" />