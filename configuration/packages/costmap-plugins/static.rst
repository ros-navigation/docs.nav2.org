.. static:

Static Layer Parameters
=======================

This implements a costmap layer taking in a map from either SLAM or ``map_server`` (or other) to place into the costmap. It resizes the costmap to its size and places the static obstacles on the planning space. 

``<static layer>`` is the corresponding plugin name selected for this type.

:``<static layer>``.enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Whether it is enabled.

:``<static layer>``.footprint_clearing_enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool False            
  ==== =======

  Description
    Clear any occupied cells under robot footprint.

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
  string "map"
  ====== =======

  Description
    Map topic to subscribe to.

  Relative topics will be relative to the node's parent namespace.
  For example, if you specify `map_topic: map` in the `static_layer` of a `global_costmap` and you launch your bringup with a `tb4` namespace:

  * User chosen namespace is `tb4`.
  * User chosen topic is `map`.
  * Topic will be remapped to `/tb4/map` without `global_costmap`.
  * Use global topic `/map` if you do not wish the node namespace to apply.
