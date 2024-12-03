.. plugin_container:

Plugin Container Layer Parameters
=================================

This implements a costmap layer which combines costmap layers within a submap, which can then be integrated with other submaps in the same parent costmap. An example would be the use of different inflation layers for different sensors or static layers  

``<plugin container layer>`` is the corresponding plugin name selected for this type.

:``<plugin container layer>``.enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Whether it is enabled.

:``<plugin container layer>``.plugins:

  ============== =======
  Type           Default
  -------------- -------
  vector<string> {}
  ============== =======

  Description
    List of mapped costmap layer names for parameter namespaces and names.

  Note
    Costmap filters are presently unsupported

