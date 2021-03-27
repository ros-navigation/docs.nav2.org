.. inflation:

Inflation Layer Parameters
==========================

``<inflation layer>`` is the corresponding plugin name selected for this type.


:``<inflation layer>``.enabled:

  ==== =======
  Type Default                                                   
  ---- -------
  bool True            
  ==== =======

  Description
    Whether it is enabled.

:``<inflation layer>``.inflation_radius:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 0.55            
  ====== =======

  Description
    Radius to inflate costmap around lethal obstacles.

:``<inflation layer>``.cost_scaling_factor:

  ====== =======
  Type   Default                                                   
  ------ -------
  double 10.0            
  ====== =======

  Description
    Exponential decay factor across inflation radius.


:``<inflation layer>``.inflate_unknown:

  ==== =======
  Type Default                                                   
  ---- -------
  bool False            
  ==== =======

  Description
    Whether to inflate unknown cells as if lethal.


:``<inflation layer>``.inflate_around_unknown:

  ==== =======
  Type Default                                                   
  ---- -------
  bool False            
  ==== =======

  Description
    Whether to inflate unknown cells.
