.. _bt_smooth_action:

SmoothPath
==========

Invokes the SmoothPath action API in the smoother server to smooth a given path plan.

Input Ports
-----------

:unsmoothed_path:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    	The blackboard variable or hard-coded input path to smooth

:max_smoothing_duration:

  ====== =======
  Type   Default
  ------ -------
  double 3.0
  ====== =======

  Description
      Maximum time to smooth for (seconds)

:check_for_collisions:

  ====== =======
  Type   Default
  ------ -------
  bool   false  
  ====== =======

  Description
    	Whether to check the output smoothed path for collisions.

:smoother_id:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The smoother plugin ID to use for smoothing in the smoother server

Output Ports
------------

:smoothed_path:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
      The output blackboard variable to assign the smoothed path to

:smoothing_duration:

  ====== =======
  Type   Default
  ------ -------
  double N/A  
  ====== =======

  Description
      The actual duration used for smoothing

:was_completed:

  ====== =======
  Type   Default
  ------ -------
  bool   N/A  
  ====== =======

  Description
      Indicates if the smoothing process was completed. Will return ``false`` if ``check_for_collisions`` is set to ``true`` and a collision is detected.

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A  
  ============== =======

  Description
    	Follow smoother error code. See ``SmoothPath`` action for the enumerated set of error code definitions.

Example
-------

.. code-block:: xml

  <SmoothPath unsmoothed_path="{path}" smoothed_path="{path}" max_smoothing_duration="3.0" smoother_id="simple_smoother" check_for_collisions="false" smoothing_duration="{smoothing_duration_used}" was_completed="{smoothing_completed}" error_code_id="{smoothing_path_error_code}"/>
