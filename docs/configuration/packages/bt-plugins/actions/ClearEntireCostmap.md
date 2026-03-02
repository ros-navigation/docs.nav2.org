<a id="bt-clear-entire-costmap-action"></a>

# ClearEntireCostmap

Action to call a costmap clearing server.

## Input Ports

* **service_name:**
  | Type   | Default   |
  |--------|-----------|
  | string | N/A       |

  Description
  : costmap service name responsible for clearing the costmap.
* **server_timeout:**
  | Type   |   Default |
  |--------|-----------|
  | double |        10 |

  Description
  : Action server timeout (ms).

## Example

```xml
<ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
