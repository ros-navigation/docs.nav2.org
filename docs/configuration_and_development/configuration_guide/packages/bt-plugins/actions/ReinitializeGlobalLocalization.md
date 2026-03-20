# ReinitializeGlobalLocalization

Used to trigger global relocalization using AMCL in case of severe delocalization or kidnapped robot problem.

## Input Ports

### **`service_name`**

| Type     | Default |
|----------|---------|
| `string` | N/A     |

Description
:   Service name.

### **`server_timeout`**

| Type     | Default |
|----------|---------|
| `double` | 10.0    |

Description
:   Server timeout (ms).

## Example

```xml
<ReinitializeGlobalLocalization service_name="reinitialize_global_localization"/>
```
