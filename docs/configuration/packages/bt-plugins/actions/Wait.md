<a id="bt-wait-action"></a>

# Wait

Invokes the Wait ROS 2 action server, which is implemented by the [nav2_behaviors](https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors) module.
This action is used in nav2 Behavior Trees as a recovery behavior.

## Input Ports

* **wait_duration:**
  | Type   |   Default |
  |--------|-----------|
  | double |         1 |

  Description
  : Wait time (s).
* **server_name:**
  | Type   | Default   |
  |--------|-----------|
  | string | N/A       |

  Description
  : Action server name.
* **server_timeout:**
  | Type   |   Default |
  |--------|-----------|
  | double |        10 |

  Description
  : Action server timeout (ms).

## Output Ports

* **error_code_id:**
  | Type   | Default   |
  |--------|-----------|
  | uint16 | N/A       |

  Description
  : Wait error code. See `Wait` action message for the enumerated set of error codes.
* **error_msg:**
  | Type   | Default   |
  |--------|-----------|
  | string | N/A       |

  Description
  : Wait error message. See `Wait` action message for the enumerated set of error codes.

## Example

```xml
<Wait wait_duration="1.0" server_name="wait_server" server_timeout="10"
      error_code_id="{wait_error_code}" error_msg="{wait_error_msg}"/>
```
