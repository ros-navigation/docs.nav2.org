.. _configuring_map_server_params:

Map Server
##########

The Map Server implements the server for handling the map load requests for the stack and hosts a map topic.

Map Server Parameters
*********************

:yaml_filename:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         N/A
  ============== =============================

  Description
    Path to map yaml file. Note from Rolling + Iron-Turtle forward: This parameter can set either from the yaml file or using the launch configuration parameter ``map``. If we set it on launch commandline / launch configuration default, we override the yaml default. If you would like the specify your map file in yaml, remove the launch default so it is not overridden in Nav2's default launch files. Before Iron: ``yaml_filename`` must be set in the yaml (even if a bogus value) so that our launch scripts can overwrite it with launch values.

:topic_name:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "map"
  ============== =============================

  Description
    Topic to publish loaded map to.

:frame_id:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "map"
  ============== =============================

  Description
    Frame to publish loaded map in.

:introspection_mode:

  ============== =============================
  Type           Default
  -------------- -----------------------------
  string         "disabled"
  ============== =============================

  Description
    The introspection mode for services and actions. Options are "disabled", "metadata", "contents".