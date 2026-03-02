<a id="bt-cancel-coverage"></a>

# CancelCoverage

Used to cancel the goals given to the complete coverage action server. The server address can be remapped using the `server_name` input port.

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
<CancelCoverage server_name="compute_complete_coverage" server_timeout="10"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
