<a id="bt-round-robin-control"></a>

# RoundRobin

Custom control flow node used to create a round-robin behavior for children BT nodes.

## Input Ports

* **wrap_around:**
  | Type   | Default   |
  |--------|-----------|
  | bool   | false     |

  Description
  : Controls wrap-around behavior. When `false`, the node returns FAILURE instead of wrapping to the first child after all children have been attempted. When `true`, the node wraps around to the first child and continues the round-robin behavior.

## Example

```xml
<RoundRobin wrap_around="false">
    <!--Add tree components here--->
</RoundRobin>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
