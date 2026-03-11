<a id="bt-distance-traveled-condition"></a>

# DistanceTraveled

Node that returns success when a configurable distance has been traveled.

## Parameters

* **transform_tolerance:**
  Defined and declared in [Behavior-Tree Navigator](../../configuring-bt-navigator.md#configuring-bt-navigator).

### Example

```yaml
bt_navigator:
  ros__parameters:
    # other bt_navigator parameters
    transform_tolerance: 0.1
```

## Input Ports

* **distance:**
  | Type   |   Default |
  |--------|-----------|
  | double |         1 |

  Description
  : The distance that must travel before returning success (m).
* **global_frame:**
  | Type   | Default   |
  |--------|-----------|
  | string | “map”     |

  Description
  : Reference frame.
* **robot_base_frame:**
  | Type   | Default     |
  |--------|-------------|
  | string | “base_link” |

  Description
  : Robot base frame.

## Example

```xml
<DistanceTraveled distance="0.8" global_frame="map" robot_base_frame="base_link"/>
```
