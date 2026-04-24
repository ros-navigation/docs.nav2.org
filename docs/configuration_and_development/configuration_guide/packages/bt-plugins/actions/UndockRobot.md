# UndockRobot { #undock-robot }

Invokes the UndockRobot ROS 2 action server, which is implemented by the docking server.

It is used to undock the robot from a docking station.

## Input Ports

### **`dock_type`**

| Type     | Default |
|----------|---------|
| `string` | N/A     |

Description
:   The dock plugin type, if not previous instance used for docking.

### **`max_undocking_time`**

| Type    | Default |
|---------|---------|
| `float` | 30.0    |

Description
:   Maximum time to get back to the staging pose.

## Output Ports

### **`success`**

| Type   | Default |
|--------|---------|
| `bool` | true    |

Description
:   If the action was successful.

### **`error_code_id`**

| Type     | Default |
|----------|---------|
| `uint16` | 0       |

Description
:   Dock robot error code. See `UndockRobot` action message for the enumerated set of error codes.

### **`error_msg`**

| Type     | Default |
|----------|---------|
| `string` | 0       |

Description
:   Dock robot error message. See `UndockRobot` action message for the enumerated set of error codes.

## Example

{% set bt_plugin_hpp_path = cache_dir + nav2_docking_hpp_path + "/undock_robot.hpp" %}

{{ render_bt_node_example(bt_plugin_hpp_path) }}
