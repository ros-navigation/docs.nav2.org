.. _configuring_costmap_filter_info_server:

Costmap Filter Info Server
###########################

The costmap filter info server is responsible for providing information about the :ref:`Costmap Filters <costmap_filters>` being used in the navigation stack. It publishes costmap filter mask specific metadata on a configured topic. This metadata is used by other components in the system to interpret the values in the costmap filter masks.

Costmap Filter Info Server Parameters
*************************************

:type:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  int            0
  ============== =============================

  Description
    Type of costmap filter used. This is an enum for the type of filter this should be interpreted as. We provide the following pre-defined types:

    - 0: keepout zones / preferred lanes filter
    - 1: speed filter, speed limit is specified in % of maximum speed
    - 2: speed filter, speed limit is specified in absolute value (m/s)
    - 3: binary filter

:filter_info_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "costmap_filter_info"
  ============== =============================

  Description
    Topic to publish costmap filter information to.

:mask_topic:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "filter_mask"
  ============== =============================

  Description
    Topic to publish filter mask to. The value of this parameter should be in accordance with ``topic_name`` parameter of Map Server tuned to filter mask publishing.

:base:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.0
  ============== =============================

  Description
    Base of ``OccupancyGrid`` mask value -> filter space value linear conversion which is being proceeded as:
    ``filter_space_value = base + multiplier * mask_value``

:multiplier:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         1.0
  ============== =============================

  Description
    Multiplier of ``OccupancyGrid`` mask value -> filter space value linear conversion which is being proceeded as:
    ``filter_space_value = base + multiplier * mask_value``

:bond_heartbeat_period:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  double         0.1
  ============== =============================

  Description
    The lifecycle node bond mechanism publishing period (on the /bond topic). Disabled if inferior or equal to 0.0.

:allow_parameter_qos_overrides:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  bool           true
  ============== =============================

  Description
    Whether to allow QoS profiles to be overwritten with parameterized values.

Example
*******
.. code-block:: yaml

    map_server:
      ros__parameters:
        yaml_filename: "turtlebot3_world.yaml"
        topic_name: "map"
        frame_id: "map"
        introspection_mode: "disabled"

    map_saver:
      ros__parameters:
        save_map_timeout: 5.0
        free_thresh_default: 0.25
        occupied_thresh_default: 0.65
        introspection_mode: "disabled"

    costmap_filter_info_server:
      ros__parameters:
        type: 1
        filter_info_topic: "costmap_filter_info"
        mask_topic: "filter_mask"
        base: 0.0
        multiplier: 0.25