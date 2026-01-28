.. _bt_compute_coverage_path_action:

ComputeCoveragePath
===================

Invokes the ComputeCoveragePath ROS 2 action server, which is implemented by the opennav_coverage_ server module.
The server address can be remapped using the ``server_name`` input port.
This server can take in both cartesian and GPS coordinates and is implemented using the ``Fields2Cover`` library.

.. _opennav_coverage: https://github.com/open-navigation/opennav_coverage

Input Ports
-----------
:generate_headland:

  ===================================== =======
  Type                                  Default
  ------------------------------------- -------
  bool                                  true
  ===================================== =======

  Description
        Whether or not to generate a headland of the field or polygon to compute coverage of

:generate_route:

  ============================================= =======
  Type                                          Default
  --------------------------------------------- -------
  bool                                          true
  ============================================= =======

  Description
        Whether or not to generate a route, e.g. an ordered set of swaths

:generate_path:

  ============== =======
  Type           Default
  -------------- -------
  bool           true
  ============== =======

  Description
        Whether or not to generate a path, e.g. adding path connectors to the ordered route

:file_field:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A
  ============== =======

  Description
        The filepath to the field's GML file to use, if not specifying the field via ``polygons``


:file_field_id:

  ============== =======
  Type           Default
  -------------- -------
  int            0
  ============== =======

  Description
        The ID of the field in the GML File to use, if multiple exist in the same file. This is the ordered number of the fields in the file.

:polygons:

  =================================== =======
  Type                                Default
  ----------------------------------- -------
  vector<geometry_msgs::msg::Polygon>  N/A
  =================================== =======

  Description
      The polygons of the field, if not specifying via a GML file. The first polygon should be the outermost region, whereas additional polygons are voids.

:polygons_frame_id:

  =================================== =======
  Type                                Default
  ----------------------------------- -------
  string                              "map"
  =================================== =======

  Description
      The polygon's frame ID, since the GML file provides the frame ID for its format, this is the frame ID for user-defined input ``polygons``.

Output Ports
------------

:nav_path:

  ========================== =======
  Type                       Default
  -------------------------- -------
  nav_msgs::msg::Path         N/A
  ========================== =======

  Description
        Path created by action server in the form of a navigation path. Takes in a blackboard variable, e.g. "{path}".

:coverage_path:

  ========================== =======
  Type                       Default
  -------------------------- -------
  vector<PathComponents>      N/A
  ========================== =======

  Description
      An ordered set of swaths and turns corresponding to the coverage path when its important to distinguish between turns and swaths for applications. A ``opennav_coverage::utils::PathComponentsIterator`` object is provided to help make this easy to use by iterating through the outputs's path components to return you the next swath and turn one at a time.

:error_code_id:

  ============== =======
  Type           Default
  -------------- -------
  uint16          N/A
  ============== =======

  Description
        Compute coverage error code. See ``ComputeCoveragePath`` action message for the enumerated set of error codes.

:error_msg:

  ============== =======
  Type           Default
  -------------- -------
  string         N/A
  ============== =======

  Description
        Compute coverage error message. See ``ComputeCoveragePath`` action message for the enumerated set of error codes.

Example
-------

.. code-block:: xml

  <ComputeCoveragePath file_field="{field_filepath}" nav_path="{path}" coverage_path="{cov_path}" server_name="compute_coverage" server_timeout="10" error_code_id="{compute_coverage_error_code}" error_msg="{compute_coverage_error_msg}"/>

Note: the blackboard IDs for the path, error code, and more may be adjusted, but need to match the corresponding parameters in the ``CoverageNavigator`` plugin to set on the blackboard for use from the action server.
