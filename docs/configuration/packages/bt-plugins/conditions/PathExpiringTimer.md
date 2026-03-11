# PathExpiringTimer

Checks if the timer has expired. Returns success if the timer has expired, otherwise it returns failure.
The timer will reset if the path gets updated.

## Input Ports

* **seconds:**
  | Type   |   Default |
  |--------|-----------|
  | double |         1 |

  Description
  : Time to check if expired.
* **path:**
  | Type                | Default   |
  |---------------------|-----------|
  | nav_msgs::msg::Path | N/A       |

  Description
  : Check if path has been updated to enable timer reset.

## Example

```xml
<PathExpiringTimer seconds="15" path="{path}"/>
```
