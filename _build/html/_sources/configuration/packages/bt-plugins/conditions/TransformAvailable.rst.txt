.. _bt_transform_available_condition:

TransformAvailable
==================

Checks if a TF transform is available. Returns failure if it cannot be found. Once found, it will always return success. Useful for initial condition checks.

Input Ports
-----------

:child:

  ====== =======
  Type   Default
  ------ -------
  string ""
  ====== =======

  Description
    	Child frame for transform.

:parent:

  ====== =======
  Type   Default
  ------ -------
  string ""
  ====== =======

  Description
    	Parent frame for transform.

Example
-------

.. code-block:: xml

  <TransformAvailable parent="odom" child="base_link"/>
