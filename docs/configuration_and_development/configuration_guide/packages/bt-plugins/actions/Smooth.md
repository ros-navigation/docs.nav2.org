# SmoothPath { #smooth-path }

Invokes the SmoothPath action API in the smoother server to smooth a given path plan.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<SmoothPath unsmoothed_path="{path}" smoothed_path="{path}" max_smoothing_duration="3.0" smoother_id="simple_smoother" check_for_collisions="false" smoothing_duration="{smoothing_duration_used}" was_completed="{smoothing_completed}" error_code_id="{smoothing_path_error_code}" error_msg="{smoothing_path_error_msg}"/>
```
