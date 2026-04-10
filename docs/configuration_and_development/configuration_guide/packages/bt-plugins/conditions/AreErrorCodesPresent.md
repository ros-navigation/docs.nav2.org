# AreErrorCodesPresent { #are-error-codes-present }

Checks the if the provided error code matches any error code within a set.

If the active error code is a match, the node returns `SUCCESS`. Otherwise, it returns `FAILURE`.

{{ render_bt_node_ports(page.title) }}

## Example

Error codes to check are defined in another port.

```xml
<AreErrorCodesPresent error_code="{error_code}" error_codes_to_check="{error_codes_to_check}"/>
```

Error codes to check are defined to be 101, 107 and 119.

```xml
<AreErrorCodesPresent error_code="{error_code}" error_codes_to_check="101;107;119"/>
```
