.. costmap_filters:

Costmap Filters Parameters
==========================

`<filter name>`: is the corresponding plugin name selected for this type.

There are following parameters common for all costmap filters plugins:

:``<filter name>``.enabled:

  ====== =======
  Type   Default
  ------ -------
  bool   True
  ====== =======

  Description
    Whether it is enabled.

:``<filter name>``.filter_info_topic:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
    Name of the CostmapFilterInfo topic having filter-related information.
