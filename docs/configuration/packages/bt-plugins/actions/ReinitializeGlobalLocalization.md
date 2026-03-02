<a id="bt-reinitialize-global-localization-action"></a>

# ReinitializeGlobalLocalization

Used to trigger global relocalization using AMCL in case of severe delocalization or kidnapped robot problem.

## Input Ports

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
<ReinitializeGlobalLocalization service_name="reinitialize_global_localization"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
