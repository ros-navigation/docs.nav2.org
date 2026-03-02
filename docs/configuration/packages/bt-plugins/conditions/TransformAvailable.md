<a id="bt-transform-available-condition"></a>

# TransformAvailable

Checks if a TF transform is available. Returns failure if it cannot be found. Once found, it will always return success. Useful for initial condition checks.

## Input Ports

* **child:**
  | Type   | Default   |
  |--------|-----------|
  | string | “”        |

  Description
  : Child frame for transform.
* **parent:**
  | Type   | Default   |
  |--------|-----------|
  | string | “”        |

  Description
  : Parent frame for transform.

## Example

```xml
<TransformAvailable parent="odom" child="base_link"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
