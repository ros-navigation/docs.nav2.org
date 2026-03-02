<a id="bt-toggle-colllsion-monitor-service"></a>

# ToggleCollisionMonitor

Calls the ToggleCollisionMonitor service. Used to toggle the collision monitor on (enabled) and off (disabled).

## Input Ports

* **enable:**
  | Type   | Default   |
  |--------|-----------|
  | bool   | true      |

  Description
  : Whether to enable or disable the collision monitor.
* **service_name:**
  | Type   | Default   |
  |--------|-----------|
  | string | N/A       |

  Description
  : Service name.
* **server_timeout:**
  | Type   |   Default |
  |--------|-----------|
  | double |        10 |

  Description
  : Server timeout (ms).

## Example

```xml
<ToggleCollisionMonitor enable="false" service_name="collision_monitor/toggle"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
