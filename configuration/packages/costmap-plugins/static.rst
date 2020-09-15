.. static:

Static Layer Parameters
=======================

``<static layer>`` is the corresponding plugin name selected for this type.

:``<static layer>``.enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Whether it is enabled.

:``<static layer>``.subscribe_to_updates:

  ==== =======
  Type Default                                                   
  ---- -------
  bool False            
  ==== =======

  Description
    Subscribe to static map updates after receiving first.

:``<static layer>``.map_subscribe_transient_local:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    QoS settings for map topic.

:``<static layer>``.transform_tolerance:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.0            
  ====== =======

  Description
    TF tolerance.

:``<static layer>``.map_topic:

  ====== =======
  Type   Default
  ------ -------
  string ""
  ====== =======

  Description
    Map topic to subscribe to. If left empty the map topic will default to the global `map_topic` parameter in `costmap_2d_ros`.