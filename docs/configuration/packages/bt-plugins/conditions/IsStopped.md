<a id="bt-is-stopped-condition"></a>

# IsStopped

BT node that tracks robot odometry and returns SUCCESS if robot is considered stopped for long enough,
RUNNING if stopped but not for long enough and FAILURE otherwise

## Input Port

* **velocity_threshold:**
  | Type   |   Default |
  |--------|-----------|
  | double |      0.01 |

  Description
  : Velocity threshold below which robot is considered stopped
* **duration_stopped:**
  | Type     |   Default |
  |----------|-----------|
  | int (ms) |      1000 |

  Description
  : Duration (ms) the velocity must remain below the threshold

## Example

```xml
<IsStopped velocity_threshold="0.01" duration_stopped="1000"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
