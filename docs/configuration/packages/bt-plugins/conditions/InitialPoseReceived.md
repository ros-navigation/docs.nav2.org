<a id="bt-initial-pose-received-condition"></a>

# InitialPoseReceived

Node that returns success when the initial pose is sent to AMCL via /initial_pose\`.

## Input Ports

* **initial_pose_received:**
  | Type   | Default                   |
  |--------|---------------------------|
  | bool   | “{initial_pose_received}” |

  Description
  : Success if the value in the port is true. Takes in a blackboard variable,
    : “{initial_pose_received}” if not specified.

## Example

```xml
<InitialPoseReceived/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
