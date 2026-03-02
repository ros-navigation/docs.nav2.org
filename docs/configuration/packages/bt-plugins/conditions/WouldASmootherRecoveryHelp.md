<a id="bt-would-a-smoother-recovery-help-condition"></a>

# WouldASmootherRecoveryHelp

Checks if the active smoother server error code is UNKNOWN, TIMEOUT, FAILED_TO_SMOOTH_PATH, or SMOOTHED_PATH_IN_COLLISION.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

## Input Port

* **error_code:**
  | Type           | Default   |
  |----------------|-----------|
  | unsigned short | N/A       |

  Description
  : The active error code to compare against. This should match the smoother server error code.

## Example

```xml
<WouldASmootherRecoveryHelp error_code="{smoother_error_code}"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
