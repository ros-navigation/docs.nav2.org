<a id="bt-would-a-planner-recovery-help-condition"></a>

# WouldAPlannerRecoveryHelp

Checks if the active planner server error code is UNKNOWN, NO_VALID_PATH, or TIMEOUT.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

## Input Port

* **error_code:**
  | Type           | Default   |
  |----------------|-----------|
  | unsigned short | N/A       |

  Description
  : The active error code to compare against. This should match the planner server error code.

## Example

```xml
<WouldAPlannerRecoveryHelp error_code="{compute_path_to_pose_error_code}"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
