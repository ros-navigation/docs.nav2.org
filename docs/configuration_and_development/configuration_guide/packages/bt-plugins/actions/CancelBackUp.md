# CancelBackUp

Used to cancel the backup action that is part of the behavior server. The server address can be remapped using the `server_name` input port.

## Input Ports

### **`service_name`**

| Type     | Default |
|----------|---------|
| `string` | N/A     |

Description
:   Service name, if not using default of `backup` due to remapping.

### **`server_timeout`**

| Type     | Default |
|----------|---------|
| `double` | 10.0    |

Description
:   Server timeout (ms).

## Example

```xml
<CancelBackUp server_name="BackUp" server_timeout="10"/>
```
