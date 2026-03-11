<a id="configuring-nav2-waypoint-follower-input-at-waypoint-plugin"></a>

# InputAtWaypoint

Lets robot to wait for external input, with timeout, at a waypoint.

## Parameters

`<nav2_waypoint_follower plugin>`: nav2_waypoint_follower plugin name defined in the **waypoint_task_executor_plugin_id** parameter in [Waypoint Follower](../configuring-waypoint-follower.md#configuring-waypoint-follower).

* **`<nav2_waypoint_follower plugin>`.enabled:**
  | Type   | Default   |
  |--------|-----------|
  | bool   | true      |

  Description
  : Whether waypoint_task_executor plugin is enabled.
* **`<nav2_waypoint_follower plugin>`.timeout:**
  | Type   |   Default |
  |--------|-----------|
  | double |        10 |

  Description
  : Amount of time in seconds to wait for user input before moving on to the next waypoint.
* **`<nav2_waypoint_follower plugin>`.input_topic:**
  | Type   | Default                   |
  |--------|---------------------------|
  | string | “input_at_waypoint/input” |

  Description
  : Topic input is published to to indicate to move to the next waypoint, in std_msgs/Empty.
* **allow_parameter_qos_overrides:**
  | Type   | Default   |
  |--------|-----------|
  | bool   | true      |

  Description
  : Whether to allow QoS profiles to be overwritten with parameterized values.
