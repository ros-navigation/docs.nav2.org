<a id="bt-cancel-follow-object"></a>

# CancelFollowObject

Used to cancel the goals given to the follow object action server. The server address can be remapped using the `server_name` input port.

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
<CancelFollowObject server_name="follow_object" server_timeout="10"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
