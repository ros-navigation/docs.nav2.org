<a id="bt-cancel-assisted-teleop"></a>

# CancelAssistedTeleop

Used to cancel the AssistedTeleop action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

## Input Ports

* **service_name:**
  | Type   | Default   |
  |--------|-----------|
  | string | N/A       |

  Description
  : Service name, if not using default of `assisted_teleop` due to remapping.
* **server_timeout:**
  | Type   |   Default |
  |--------|-----------|
  | double |        10 |

  Description
  : Server timeout (ms).

## Example

```xml
<CancelAssistedTeleop server_name="assisted_teleop" server_timeout="10"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
